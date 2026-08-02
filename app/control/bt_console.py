"""Bluetooth command console (runs on the UNO Q MPU, alongside app.main).

Uses the standard library's socket.AF_BLUETOOTH / BTPROTO_RFCOMM (Linux only,
needs a bluetooth-enabled Python build -- true for Debian's stock python3) so
no extra dependency is needed, matching the rest of this app's preference for
stdlib transports (see camera.py's urllib note in docs/architecture.md).

Setup (once, on the board):
    sudo apt install bluetooth bluez
    bluetoothctl
      power on
      discoverable on
      pairable on
      agent on
      (pair from your phone/laptop as usual)

Then connect with any Bluetooth serial-terminal app (RFCOMM channel 1) and
send one command per line:
    start  -- launch the evasion loop (app.main.run) in the background
    stop   -- stop the evasion loop and halt the motors
    usb    -- list USB devices (lsusb) and serial ports, to help pick
              --front-port/--rear-port
"""

import argparse
import glob
import socket
import subprocess
import threading

from .. import config
from .. import main as evasion_main

RFCOMM_CHANNEL = 1


class BluetoothConsole:
    def __init__(self, model_path, front_port, rear_port, log_path, backend, stub=False):
        self._model_path = model_path
        self._front_port = front_port
        self._rear_port = rear_port
        self._log_path = log_path
        self._backend = backend
        self._stub = stub
        self._run_thread = None
        self._stop_event = threading.Event()

    def _handle(self, cmd):
        cmd = cmd.strip().lower()
        if cmd == "start":
            return self._start()
        if cmd == "stop":
            return self._stop()
        if cmd == "usb":
            return self._usb()
        return f"unknown command: {cmd!r} (try start/stop/usb)"

    def _start(self):
        if self._run_thread is not None and self._run_thread.is_alive():
            return "already running"
        self._stop_event.clear()
        self._run_thread = threading.Thread(
            target=evasion_main.run,
            kwargs=dict(
                model_path=self._model_path,
                front_port=self._front_port,
                rear_port=self._rear_port,
                log_path=self._log_path,
                stub=self._stub,
                backend=self._backend,
                stop_event=self._stop_event,
            ),
            daemon=True,
        )
        self._run_thread.start()
        return "started"

    def _stop(self):
        if self._run_thread is None or not self._run_thread.is_alive():
            return "not running"
        self._stop_event.set()
        self._run_thread.join(timeout=5.0)
        return "stopped" if not self._run_thread.is_alive() else "stop requested (still shutting down)"

    def _usb(self):
        lines = []
        try:
            result = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5.0)
            lines.append(result.stdout.strip() or "(lsusb: no output)")
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            lines.append(f"lsusb unavailable: {exc}")
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        lines.append("serial ports: " + (", ".join(ports) if ports else "(none)"))
        return "\n".join(lines)

    def serve_forever(self):
        server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server.bind(("", RFCOMM_CHANNEL))
        server.listen(1)
        _advertise_sdp()
        print(f"[bt_console] listening on RFCOMM channel {RFCOMM_CHANNEL}")
        try:
            while True:
                conn, addr = server.accept()
                print(f"[bt_console] connected: {addr}")
                self._serve_client(conn)
        finally:
            server.close()

    def _serve_client(self, conn):
        buf = b""
        try:
            with conn:
                conn.sendall(b"evasion-bot ready. commands: start stop usb\n")
                while True:
                    chunk = conn.recv(256)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        cmd = line.decode(errors="ignore").strip()
                        if not cmd:
                            continue
                        reply = self._handle(cmd)
                        conn.sendall((reply + "\n").encode())
        except OSError as exc:
            print(f"[bt_console] connection error: {exc}")


def _advertise_sdp():
    """Best-effort SDP registration (via bluez-utils' sdptool, if installed)
    so phones can discover the serial port service by name. Harmless no-op
    otherwise -- pair manually and connect to RFCOMM_CHANNEL directly."""
    try:
        subprocess.run(
            ["sdptool", "add", f"--channel={RFCOMM_CHANNEL}", "SP"],
            capture_output=True, timeout=5.0, check=False,
        )
    except FileNotFoundError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="models/person_detect.tflite")
    ap.add_argument("--front-port", default="/dev/ttyUSB0")
    ap.add_argument("--rear-port", default="/dev/ttyUSB1")
    ap.add_argument("--log", default="run.csv")
    ap.add_argument("--stub", action="store_true",
                     help="run the evasion loop with no hardware when 'start' is sent")
    ap.add_argument("--backend", choices=["bbox", "pose"], default=config.DETECTOR_BACKEND)
    args = ap.parse_args()
    console = BluetoothConsole(args.model, args.front_port, args.rear_port,
                                args.log, args.backend, stub=args.stub)
    console.serve_forever()


if __name__ == "__main__":
    main()
