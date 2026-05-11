import asyncio
import pyshark
import argparse
from build_html_report import print_bilan
from trasmitter_tests import *
from enum import Enum

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

Tests_State = {
    "no_ack_test_tx": False,
    "ack_test_tx": False,
    "no_frame_pending_test_tx": False,
    "frame_pending_test_tx": False,
    "short_addr": False,
    "long_addr": False,
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
            case _:
                print(f"[⚠]Unknown test with seq_no: {wpan['wpan_wpan_seq_no']}")
    except KeyError:
        pass  # Paquet sans layer wpan, on ignore

import argparse
import subprocess
import json
import threading

def kill_proc(proc):
    print("[!] Timeout d'inactivité → arrêt")
    proc.terminate()

TIMEOUT = 10 # secondes

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Interface de capture")
    parser.add_argument("--channel", type=int, default=15)
    args = parser.parse_args()

    cmd = [
        "tshark",
        "-i", args.interface,
        "-Y", "wpan",
        "-T", "ek",
        "-l",
    ]

    print(f"[+] Capture live sur {args.interface}...")
    print(f"[+] {' '.join(cmd)}")

    proc = subprocess.Popen(
        cmd,
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
        timer = threading.Timer(TIMEOUT, kill_proc, args=(proc,))
        timer.daemon = True
        timer.start()

    try:
        for line in proc.stdout:
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
        proc.terminate()

    print_bilan(Tests_State)

if __name__ == "__main__":
    main()