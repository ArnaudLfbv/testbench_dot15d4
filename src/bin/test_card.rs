#![no_std]
#![no_main]

#[cfg(feature = "tsch")]
use dot15d4::mac::procedures::{join_network_from_scan, scan, tsch_start_pan};
use dot15d4::{
    RngCore, RngError, driver::{
        DriverEventChannel, DriverEventReceiver, DriverEventSender, DriverRequestChannel, DriverRequestReceiver, DriverRequestSender, DriverService, radio::{
            RadioDriver, frame::{Address, AddressingMode, IeRepr, IeReprList}, tasks::{RadioDriverApi, TaskOff}
        }, socs::nrf::{NrfRadioDriver, NrfRadioSleepTimer, export::pac::ficr::info}, timer::{NsDuration, RadioTimerApi}
    }, mac::{
        MAC_BUFFER_SIZE, MacBufferAllocator, MacIndicationChannel, MacIndicationReceiver, MacIndicationSender, MacRequestChannel, MacRequestReceiver, MacRequestSender, MacService, frame::mpdu::{self, beacon_frame, data_frame}, primitives::{BeaconRequest, DataRequest, MacRequest}, procedures::{get_coordinator_extended_address, get_device_extended_address}
    }, scheduler::{
        SchedulerRequestChannel, SchedulerRequestReceiver, SchedulerRequestSender, SchedulerService,
    }, util::{allocator::IntoBuffer, buffer_allocator, info}
};
#[cfg(feature = "executor-trace")]
use dot15d4_examples_nrf52840::gpio_trace::PIN_EXECUTOR;
#[cfg(feature = "radio-trace")]
use dot15d4_examples_nrf52840::radio_tracing_config;
use dot15d4_examples_nrf52840::{config_peripherals, AvailableResources};
use embassy_executor::Spawner;
use static_cell::StaticCell;

static MAC_REQUEST_CHANNEL: StaticCell<MacRequestChannel> = StaticCell::new();
static SCHEDULER_REQUEST_CHANNEL: StaticCell<SchedulerRequestChannel> = StaticCell::new();
static DRIVER_REQUEST_CHANNEL: StaticCell<DriverRequestChannel> = StaticCell::new();
static DRIVER_EVENT_CHANNEL: StaticCell<DriverEventChannel> = StaticCell::new();
static MAC_INDICATION_CHANNEL: StaticCell<MacIndicationChannel> = StaticCell::new();

// TODO: use PNRG from device
#[derive(Debug, Clone, Copy, Default)]
pub struct FakeRng;

impl RngCore for FakeRng {
    fn next_u32(&mut self) -> u32 {
        3
    }
    fn next_u64(&mut self) -> u64 {
        3
    }
    fn fill_bytes(&mut self, d: &mut [u8]) {
        d.fill(0);
    }
    fn try_fill_bytes(&mut self, d: &mut [u8]) -> Result<(), RngError> {
        d.fill(0);
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "rtos-trace")]
    let start_tracing = dot15d4::util::trace::instrument!(embassy cpu_freq: 64_000_000 Hz);
    let AvailableResources { radio, timer, .. } = config_peripherals(
        #[cfg(feature = "rtos-trace")]
        start_tracing,
    );

    let radio = RadioDriver::new(
        radio,
        timer,
        #[cfg(feature = "executor-trace")]
        executor_trace_channel,
        #[cfg(feature = "radio-trace")]
        radio_tracing_config(),
    );

    let buffer_allocator = buffer_allocator!(MAC_BUFFER_SIZE, 10);
    let mac_request_channel = MAC_REQUEST_CHANNEL.init(MacRequestChannel::new());
    let scheduler_request_channel = SCHEDULER_REQUEST_CHANNEL.init(SchedulerRequestChannel::new());
    let driver_request_channel = DRIVER_REQUEST_CHANNEL.init(DriverRequestChannel::new());
    let driver_response_channel = DRIVER_EVENT_CHANNEL.init(DriverEventChannel::new());
    let mac_indication_channel = MAC_INDICATION_CHANNEL.init(MacIndicationChannel::new());

    let ieee802154_address = radio.ieee802154_address();

    spawner.spawn(
        driver_service_task(
            radio,
            buffer_allocator,
            driver_request_channel.receiver(),
            driver_response_channel.sender(),
        )
        .unwrap(),
    );
    spawner.spawn(
        scheduler_service_task(
            timer,
            scheduler_request_channel.receiver(),
            driver_request_channel.sender(),
            driver_response_channel.receiver(),
            buffer_allocator,
            ieee802154_address,
        )
        .unwrap(),
    );
    spawner.spawn(
        mac_service_task(
            timer,
            buffer_allocator,
            mac_request_channel.receiver(),
            scheduler_request_channel.sender(),
            mac_indication_channel.sender(),
        )
        .unwrap(),
    );

    upper_layer_task(
        timer,
        buffer_allocator,
        mac_request_channel.sender(),
        mac_indication_channel.receiver(),
    )
    .await;
}

#[embassy_executor::task]
async fn mac_service_task(
    timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    mac_request_receiver: MacRequestReceiver<'static>,
    scheduler_request_sender: SchedulerRequestSender<'static>,
    mac_indication_sender: MacIndicationSender<'static>,
) {
    let mut mac_service = MacService::<NrfRadioDriver>::new(
        timer,
        buffer_allocator,
        mac_request_receiver,
        mac_indication_sender,
        scheduler_request_sender,
    );
    mac_service.run().await
}

#[embassy_executor::task]
async fn scheduler_service_task(
    timer: NrfRadioSleepTimer,
    scheduler_request_receiver: SchedulerRequestReceiver<'static>,
    driver_request_sender: DriverRequestSender<'static>,
    driver_response_receiver: DriverEventReceiver<'static>,
    buffer_allocator: MacBufferAllocator,
    address: [u8; 8],
) -> ! {
    let mut rng = FakeRng;
    let mut scheduler_service = SchedulerService::<NrfRadioDriver>::new(
        timer,
        scheduler_request_receiver,
        driver_request_sender,
        driver_response_receiver,
        buffer_allocator,
        &mut rng,
        &address,
    );

    scheduler_service.run().await
}

#[embassy_executor::task]
async fn driver_service_task(
    radio: RadioDriver<NrfRadioDriver, TaskOff>,
    buffer_allocator: MacBufferAllocator,
    driver_request_receiver: DriverRequestReceiver<'static>,
    driver_response_sender: DriverEventSender<'static>,
) -> ! {
    #[cfg(feature = "executor-trace")]
    let executor_trace_channel = PIN_EXECUTOR.gpiote_channel as usize;

    let driver_service = DriverService::new(
        radio,
        driver_request_receiver,
        driver_response_sender,
        buffer_allocator,
    );

    driver_service.run().await
}

fn data_request(
    buffer_allocator: MacBufferAllocator,
    seq_nr: u8,
    src_addr: &Address<&[u8]>,
    dst_addr: &Address<&[u8]>,
    payload: &[u8],
) -> MacRequest {
    static IES: [IeRepr; 1] = [IeRepr::TschSynchronizationNestedIe];
    static IE_REPR_LIST: IeReprList<'static, IeRepr> = IeReprList::new(&IES);
    let mut mpdu_writer =
        data_frame::<NrfRadioDriver>(Some(IE_REPR_LIST), payload.len() as u16, buffer_allocator)
            .unwrap();

    mpdu_writer.set_sequence_number(seq_nr);
    mpdu_writer.set_ack_request(true);

    let mut addressing = mpdu_writer.addressing_fields_mut();
    addressing.src_address_mut().set(src_addr);
    addressing.dst_address_mut().set(dst_addr);

    mpdu_writer.frame_payload_mut().copy_from_slice(payload);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_writer.into_mpdu_frame(),
    })
}

async fn upper_layer_task(
    mut timer: NrfRadioSleepTimer,
    buffer_allocator: MacBufferAllocator,
    request_sender: MacRequestSender<'static>,
    mac_indication_receiver: MacIndicationReceiver<'static>,
) -> ! {
    info!("Start as client");

    info!("Start as coordinator");
    tsch_start_pan(&request_sender, timer).await;

    let dst_addr = get_coordinator_extended_address(&request_sender).await;
    let src_addr = get_device_extended_address(&request_sender).await;

    let short_dst_addr = dst_addr.try_into_short_address().unwrap();
    let short_src_addr = src_addr.try_into_short_address().unwrap();

    let instant = timer.now();
    let mut nb_sent = 0;

    // unsafe {
    //     timer
    //         .wait_until(instant + nb_sent * NsDuration::millis(2000))
    //         .await
    //         .unwrap()
    // };
    loop {
        let data_request =
                run_tests(nb_sent, buffer_allocator, &src_addr, &dst_addr);

        let request_token = request_sender.allocate_request_token().await;

        let mac_confirm = request_sender
            .send_request_awaiting_response(request_token, data_request)
            .await;
        nb_sent += 1;

        match mac_confirm {
            dot15d4::mac::primitives::MacConfirm::McpsData(timestamp) => {
                #[cfg(any(feature = "log", feature = "defmt"))]
                {
                    let timestamp = timestamp.unwrap().ticks();
                    info!("Tx Timestamp: {}", timestamp);
                }
                unsafe {
                    timer
                        .wait_until(instant + nb_sent * NsDuration::millis(2000))
                        .await
                        .unwrap()
                };
            }
            _ => unreachable!(),
        }

        if nb_sent >= 10 {
            info!("Tests done, wait for incoming frames");
            break;
        }
    }
    let mut consumer_token = mac_indication_receiver
        .try_allocate_consumer_token()
        .unwrap();
    loop {
        let (response_token, mac_indication) = mac_indication_receiver
            .receive_request_async(&mut consumer_token, &())
            .await;
        match mac_indication {
            dot15d4::mac::primitives::MacIndication::McpsData(data_indication) => {
                let received_mpdu = data_indication.mpdu;
                #[cfg(any(feature = "log", feature = "defmt"))]
                {
                    let timestamp = data_indication.timestamp.ticks();
                    info!("Rx Timestamp : {}", timestamp);
                }
                unsafe { buffer_allocator.deallocate_buffer(received_mpdu.into_buffer()) };
                mac_indication_receiver.received(response_token, ());
            }
            _ => unreachable!(),
        }
    }
}

// Tests frames will be distinct by the sequence number.
enum TestId {
    NoAckTestTx = 0,
    AckTestTx = 1,
    NoFramePendingTestTx = 2,
    FramePendingTestTx = 3,
    ShortSrcAddrTestTx = 4,
    LongSrcAddrTestTx = 5,
    NoPanIdCompressionTestTx = 6,
    PanIdCompressionTestTx = 7,
    SecurityEnabledTestTx = 8,
    SecurityDisabledTestTx = 9,
}

struct TestFrameConfig {
    test_id: TestId,
    ack_request: bool,
    frame_pending: bool,
    addressing_mode: AddressingMode,
    pan_id_compression: bool,
    security_enabled: bool,
}

impl Default for TestFrameConfig {
    fn default() -> Self {
        Self {
            test_id: TestId::NoAckTestTx,
            ack_request: false,
            frame_pending: false,
            addressing_mode: AddressingMode::Extended,
            pan_id_compression: false,
            security_enabled: false,
        }
    }
}

impl From<u32> for TestId {
    fn from(value: u32) -> Self {
        match value {
            0 => TestId::NoAckTestTx,
            1 => TestId::AckTestTx,
            2 => TestId::NoFramePendingTestTx,
            3 => TestId::FramePendingTestTx,
            4 => TestId::ShortSrcAddrTestTx,
            5 => TestId::LongSrcAddrTestTx,
            6 => TestId::NoPanIdCompressionTestTx,
            7 => TestId::PanIdCompressionTestTx,
            8 => TestId::SecurityEnabledTestTx,
            9 => TestId::SecurityDisabledTestTx,
            _ => unimplemented!(),
        }
    }
}

const TEST_PAYLOAD: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

fn build_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>, 
    config: TestFrameConfig
) -> MacRequest {
    let mut mpdu_writer =
        data_frame::<NrfRadioDriver>(None, TEST_PAYLOAD.len() as u16, buffer_allocator).unwrap();
    mpdu_writer.set_sequence_number(config.test_id as u8);
    let mut addressing = mpdu_writer.addressing_fields_mut();
    addressing.src_address_mut().set(src_addr);
    addressing.dst_address_mut().set(dst_addr);
    mpdu_writer.frame_payload_mut().copy_from_slice(&TEST_PAYLOAD);

    mpdu_writer.set_ack_request(config.ack_request);
    mpdu_writer.set_frame_pending(config.frame_pending);

    let mut mpdu_frame = mpdu_writer.into_mpdu_frame();
    let mut fc = mpdu_frame.frame_control_mut();

    fc.set_src_addressing_mode(config.addressing_mode);
    fc.set_dst_addressing_mode(config.addressing_mode);
    fc.set_pan_id_compression(config.pan_id_compression);
    fc.set_security_enabled(config.security_enabled);

    MacRequest::McpsData(DataRequest {
        mpdu: mpdu_frame,
    })
}

fn no_ack_test_frame<'a>(
    buffer_allocator: MacBufferAllocator,
    src_addr: &Address<[u8; 8]>,
    dst_addr: &Address<[u8; 8]>,
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::NoAckTestTx,
        ack_request: false,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn ack_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::AckTestTx,
        ack_request: true,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn no_frame_pending_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::NoFramePendingTestTx,
        frame_pending: false,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn frame_pending_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::FramePendingTestTx,
        frame_pending: true,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn short_src_addr_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::ShortSrcAddrTestTx,
        addressing_mode: AddressingMode::Short,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn long_src_addr_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::LongSrcAddrTestTx,
        addressing_mode: AddressingMode::Extended,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn no_pan_id_compression_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::NoPanIdCompressionTestTx,
        pan_id_compression: false,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn pan_id_compression_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::PanIdCompressionTestTx,
        pan_id_compression: true,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn security_enabled_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::SecurityEnabledTestTx,
        security_enabled: true,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn security_disabled_test_frame(
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>
) -> MacRequest {
    let config = TestFrameConfig {
        test_id: TestId::SecurityDisabledTestTx,
        security_enabled: false,
        ..Default::default()
    };
    build_test_frame(buffer_allocator, src_addr, dst_addr, config)
}

fn run_tests(nb_sent: u32, 
    buffer_allocator: MacBufferAllocator, 
    src_addr: &Address<[u8; 8]>, 
    dst_addr: &Address<[u8; 8]>) 
    -> MacRequest {
    match TestId::from(nb_sent) {
        TestId::NoAckTestTx => no_ack_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::AckTestTx => ack_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::NoFramePendingTestTx => no_frame_pending_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::FramePendingTestTx => frame_pending_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::ShortSrcAddrTestTx => short_src_addr_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::LongSrcAddrTestTx => long_src_addr_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::NoPanIdCompressionTestTx => no_pan_id_compression_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::PanIdCompressionTestTx => pan_id_compression_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::SecurityEnabledTestTx => security_enabled_test_frame(buffer_allocator, src_addr, dst_addr),
        TestId::SecurityDisabledTestTx => security_disabled_test_frame(buffer_allocator, src_addr, dst_addr),
        _ => unimplemented!(),
    }
}
