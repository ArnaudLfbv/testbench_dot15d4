import asyncio
import pyshark
import argparse
from build_html_report import print_bilan
from trasmitter_tests import *
from enum import Enum
from scapy.all import wrpcap
from datetime import datetime, time

# Correction pour Python 3.10+
asyncio.set_event_loop(asyncio.new_event_loop())

def get_interfaces():
    print(pyshark.LiveCapture().interfaces)

# Pour ajouter un test, il suffit de :
# 1. Ajouter une nouvelle entrée dans l'enum TestEnum
# 2. Ajouter une nouvelle fonction de test dans le fichier correspondant (ex: trasmitter_tests.py)
# 3. Ajouter un nouveau case dans la fonction process_frame() pour appeler la fonction de test correspondante

class TestEnum(Enum):
    NoAckTestTx = 0
    AckTestTx = 1
    NoFramePendingTx = 2
    FramePendingTx = 3
    ShortAddr = 4
    LongAddr = 5
    NoPanIdCompression = 6
    PanIdCompression = 7
    SecurityEnabled = 8
    SecurityDisabled = 9

Tests_State = {
    "no_ack_test_tx": False,
    "ack_test_tx": False,
    "no_frame_pending_test_tx": False,
    "frame_pending_test_tx": False,
    "short_addr": False,
    "long_addr": False,
    "no_pan_id_compression": False,
    "pan_id_compression": False,
    "security_enabled": False,
    "security_disabled": False,
}

def process_frame(pkt):
    try:
        wpan = pkt['wpan']
        if not wpan['wpan_wpan_seq_no']:
            print("[!] Packet missing seq_no")
            return

        match int(wpan['wpan_wpan_seq_no']):
            case TestEnum.NoAckTestTx.value:
                no_ack_response_test(pkt, Tests_State)
            case TestEnum.AckTestTx.value:
                ack_response_test(pkt, Tests_State)
            case TestEnum.NoFramePendingTx.value:
                no_frame_pending_test(pkt, Tests_State)
            case TestEnum.FramePendingTx.value:
                frame_pending_test(pkt, Tests_State)
            case TestEnum.ShortAddr.value:
                short_addr_test(pkt, Tests_State)
            case TestEnum.LongAddr.value:
                long_addr_test(pkt, Tests_State)
            case TestEnum.NoPanIdCompression.value:
                no_pan_id_compression_test(pkt, Tests_State)
            case TestEnum.PanIdCompression.value:
                pan_id_compression_test(pkt, Tests_State)
            case TestEnum.SecurityEnabled.value:
                security_enabled_test(pkt, Tests_State)
            case TestEnum.SecurityDisabled.value:
                security_disabled_test(pkt, Tests_State)
            case _:
                print(f"[⚠]Unknown test with seq_no: {wpan['wpan_wpan_seq_no']}")
    except KeyError:
        pass  # Paquet sans layer wpan, on ignore


# =========================================================================
# Capture sans enregistrement sur fichier Wireshark 

import argparse
import subprocess
import json
import threading

def kill_proc(proc):
    print("[!] Timeout d'inactivité → arrêt")
    proc.terminate()

TIMEOUT = 10 # secondes

def process_live():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Interface de capture")
    parser.add_argument("--channel", type=int, default=15)
    args = parser.parse_args()

    now = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Python cmd qui envoie les données vers le fichier python
    python_cmd = [
        "tshark",
        "-i", args.interface,
        "-Y", "wpan",
        "-T", "ek",
        "-l",
    ]

    print(f"[+] Capture live sur {args.interface}...")
    print(f"[+] {' '.join(python_cmd)}")

    python_proc = subprocess.Popen(
        python_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    timer = None

    def reset_timer():
        nonlocal timer
        if timer:
            timer.cancel()
        timer = threading.Timer(TIMEOUT, kill_proc, args=(python_proc,))
        timer.daemon = True
        timer.start()

    try:
        for line in python_proc.stdout:
            line = line.strip()
            if not line:
                continue
            reset_timer()

            try:
                packet_json = json.loads(line)
                if "layers" in packet_json:
                    process_frame(packet_json['layers'])

            except json.JSONDecodeError:
                print(f"[!] JSON decode error: {line}")
                pass

    except KeyboardInterrupt:
        print("\n[!] Arrêt de la capture.")

    finally:
        print("[+] Fin de la capture.")
        python_proc.terminate()

    print_bilan(Tests_State, now)


# =========================================================================
# Capture avec enregistrement sur Wireshark 
# MAIS problème, il manque une ou deux trames, au début et/ou à la fin

def process_delayed_with_wireshark_v1():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Interface de capture")
    parser.add_argument("--channel", type=int, default=15)
    args = parser.parse_args()

    now = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Python cmd qui envoie les données vers le fichier python
    python_cmd = [
        "bash",
        "./tshark_timeout.sh",
        now,
    ]

    isWireSharkFinished = False

    print(f"[+] Capture live sur {args.interface}...")
    print(f"[+] {' '.join(python_cmd)}")

    python_proc = subprocess.Popen(
        python_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    timeElapsed = 0
    import time
    while not isWireSharkFinished:
        if python_proc.poll() is None:
            time.sleep(1)
            timeElapsed += 1
            pass
        else:
            isWireSharkFinished = True
        print("Alive")
        print(isWireSharkFinished)
        print(python_proc.poll())
        print(timeElapsed)

    try:
        def test(pkt):
            print(pkt)

        capture = pyshark.FileCapture(f"capture_{now}.pcapng", display_filter="wpan")
        capture.apply_on_packets(test)

    except KeyboardInterrupt:
        print("\n[!] Arrêt de la capture.")

    finally:
        print("[+] Fin de la capture.")
        python_proc.terminate()

    print_bilan(Tests_State, now)


import os
def process_delayed_with_wireshark():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Interface de capture")
    parser.add_argument("--channel", type=int, default=15)
    args = parser.parse_args()

    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    wiresharkFile = f"{now}_capture.pcapng"

    # Python cmd qui envoie les données vers le fichier python
    python_cmd = [
        "tshark", 
        "-i", args.interface, 
        "-w", wiresharkFile
    ]

    print(f"[+] Capture live sur {args.interface}...")
    print(f"[+] {' '.join(python_cmd)}")

    python_proc = subprocess.Popen(
        python_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    print("[+] Reset")
    subprocess.call(["probe-rs", "reset", "--chip", "nRF52840_xxAA"])

    try:
        # -- Timeout logic --
        print("[+] Timeout started")
        isWireSharkFinished = False
        timeElapsedSinceLastCom = 0
        fileSize = 0
        import time
        time.sleep(10)
        while not isWireSharkFinished:
            if fileSize == os.path.getsize(wiresharkFile):
                timeElapsedSinceLastCom += 1
            else:
                timeElapsedSinceLastCom = 0
                fileSize = os.path.getsize(wiresharkFile)
            if timeElapsedSinceLastCom >= 10:
                isWireSharkFinished = True
            print(timeElapsedSinceLastCom)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[!] Arrêt de la capture.")

    finally:
        print("[+] Fin de la capture.")
        python_proc.terminate()

    def test(pkt):
        print(pkt)

    capture = pyshark.FileCapture(wiresharkFile, display_filter="wpan")
    capture.apply_on_packets(test)

    print_bilan(Tests_State, now)

def main():
    # process_live()
    # process_delayed_with_wireshark_v1()
    process_delayed_with_wireshark()

if __name__ == "__main__":
    main()