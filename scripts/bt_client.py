"""Talk to app.control.bt_console from a Linux PC over RFCOMM.

Deliberately avoids rfcomm/hcitool/sdptool (bluez-utils' legacy CLI tools,
deprecated upstream and missing on plenty of distros) in favor of the same
stdlib socket.AF_BLUETOOTH/BTPROTO_RFCOMM approach bt_console.py's server
side already uses -- one fewer thing that needs to be installed to test the
Bluetooth console. Pair the robot first (see README's Bluetooth section);
this script only opens the RFCOMM connection, it doesn't pair.

    python -m scripts.bt_client 14:B5:CD:EA:BB:09

Type start/stop/usb and press enter; Ctrl-C or Ctrl-D to quit.
"""

import argparse
import socket
import sys
import threading


def _print_replies(conn):
    buf = b""
    while True:
        chunk = conn.recv(256)
        if not chunk:
            print("[bt_client] connection closed by robot")
            break
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            print(line.decode(errors="ignore"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mac", help="robot's Bluetooth MAC address, e.g. 14:B5:CD:EA:BB:09")
    ap.add_argument("--channel", type=int, default=1,
                     help="RFCOMM channel (matches bt_console.RFCOMM_CHANNEL, default 1)")
    args = ap.parse_args()

    conn = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    conn.connect((args.mac, args.channel))
    print(f"[bt_client] connected to {args.mac} channel {args.channel}")

    threading.Thread(target=_print_replies, args=(conn,), daemon=True).start()

    try:
        for line in sys.stdin:
            cmd = line.strip()
            if not cmd:
                continue
            conn.sendall((cmd + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        conn.close()


if __name__ == "__main__":
    main()
