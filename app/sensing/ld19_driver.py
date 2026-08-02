"""Self-contained driver for the LDRobot LD19 360-degree LiDAR.

No lidar library dependency -- just pyserial and the documented protocol.
The LD19 needs no commands at all: from power-on it streams 47-byte packets
on TX at 230400 8N1, and we only ever listen. Each packet:

    offset  size  field
    0       1     header, always 0x54
    1       1     ver_len, always 0x2C (low 5 bits = 12 points per packet)
    2       2     rotation speed, deg/s, little-endian
    4       2     start angle, 0.01 deg units, LE
    6       36    12 x measurement: distance mm (u16 LE) + intensity (u8)
    42      2     end angle, 0.01 deg units, LE
    44      2     timestamp, ms, LE
    46      1     CRC8 over bytes 0..45 (poly 0x4D, MSB-first, init 0)

Point angles are linearly interpolated between start and end (11 equal
intervals for 12 points), wrapping through 360. The sensor spins CLOCKWISE
viewed from the top, so its native angle runs CW -- the opposite of the
robot-frame convention (CCW positive) used by sensing/lidar.py and all the
geometry. The driver converts to CCW at the source (see LD19_NATIVE_CW in
config) so every consumer downstream can keep thinking in one convention.

Layers, separated so the protocol logic unit-tests without hardware:

    crc8() / PacketParser   bytes in, validated packets out; resyncs on
                            garbage by hunting for the next 0x54 header
    RevolutionAssembler     packets in, complete 360-degree revolutions of
                            Point(angle_deg, distance_m, intensity) out
    LD19                    owns the serial port + a reader thread; keeps
                            the freshest complete revolution + health stats

Identifying WHICH physical unit a port is: both our CP2102 adapters report
identical USB serial numbers ("0001"), so /dev/ttyUSB0 vs /dev/ttyUSB1 is
luck of the enumeration draw at boot. Use the physical port path instead
(/dev/serial/by-path/...), which is stable as long as the plugs stay in the
same hub sockets -- scripts/lidar_test.py --identify walks through
assigning them and prints the exact config lines to paste.
"""

import struct
import threading
import time
from collections import namedtuple

from .. import config

# Bump when the driver changes behaviourally. Lets the bench confirm which
# copy is actually loaded:  python3 -c "from app.sensing import ld19_driver;
# print(ld19_driver.__version__)"  -- if this errors or prints an older
# value, the running tree is stale (clear __pycache__ / re-sync).
__version__ = "2026.07.30-caught-debounce"

# pyserial is only needed to talk to real hardware; the parser/assembler
# below must stay importable on dev machines without it (for the tests).
try:
    import serial
    from serial.tools import list_ports
except ImportError:                                   # pragma: no cover
    serial = None
    list_ports = None

Point = namedtuple("Point", ["angle_deg", "distance_m", "intensity"])
Packet = namedtuple("Packet", ["speed_dps", "start_deg", "end_deg",
                               "timestamp_ms", "measurements"])

PACKET_LEN = 47
HEADER = 0x54
VER_LEN = 0x2C          # 12 measurements per packet
POINTS_PER_PACKET = 12

# ---------------------------------------------------------------------------
# CRC8, polynomial 0x4D, MSB-first, init 0 -- as specified in the LD19
# development manual. Generated rather than hard-coded: a typo in a 256-entry
# table is invisible until it rejects every packet.
# ---------------------------------------------------------------------------


def _make_crc_table(poly=0x4D):
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) if (c & 0x80) else (c << 1)
        table.append(c & 0xFF)
    return table


_CRC_TABLE = _make_crc_table()


def crc8(data):
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


# ---------------------------------------------------------------------------
# Byte stream -> packets
# ---------------------------------------------------------------------------


class PacketParser:
    """Feed it raw serial bytes; it yields validated Packets.

    Tolerant by design: serial streams begin mid-packet, drop bytes, and
    carry line noise. On any malformed or CRC-failing candidate we advance
    ONE byte and hunt for the next 0x54 -- never discard a whole buffer,
    which could throw away a good packet straddling the junk.
    """

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, data):
        self._buf.extend(data)
        packets = []
        while len(self._buf) >= PACKET_LEN:
            if self._buf[0] != HEADER or self._buf[1] != VER_LEN:
                del self._buf[0]
                self.resyncs += 1
                continue
            candidate = bytes(self._buf[:PACKET_LEN])
            if crc8(candidate[:-1]) != candidate[-1]:
                self.crc_errors += 1
                del self._buf[0]      # not a real packet boundary: slide on
                continue
            packets.append(self._unpack(candidate))
            del self._buf[:PACKET_LEN]
        return packets

    @staticmethod
    def _unpack(raw):
        speed, start = struct.unpack_from("<HH", raw, 2)
        measurements = []
        for i in range(POINTS_PER_PACKET):
            dist_mm, intensity = struct.unpack_from("<HB", raw, 6 + 3 * i)
            measurements.append((dist_mm, intensity))
        end, ts = struct.unpack_from("<HH", raw, 42)
        return Packet(speed_dps=speed,
                      start_deg=start / 100.0,
                      end_deg=end / 100.0,
                      timestamp_ms=ts,
                      measurements=measurements)


# ---------------------------------------------------------------------------
# Packets -> whole revolutions
# ---------------------------------------------------------------------------


class RevolutionAssembler:
    """Accumulates packets until a full 360 degrees has swept past, then
    emits the revolution as a list of Points in the LOCAL frame downstream
    code expects: degrees in [0, 360), CCW positive.

    Zero-distance measurements ("no return within range") are dropped here,
    at the source: a raw 0.0 m would otherwise win every nearest-obstacle
    reduction in sensing/lidar.py and convince the robot it is permanently
    cornered.
    """

    def __init__(self, native_cw=None):
        self._native_cw = (config.LD19_NATIVE_CW
                          if native_cw is None else native_cw)
        self._points = []
        self._swept = 0.0
        self._last_angle = None

    def feed(self, packet):
        """Returns a completed revolution (list[Point]) or None."""
        done = None
        span = (packet.end_deg - packet.start_deg) % 360.0
        step = span / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

        for i, (dist_mm, intensity) in enumerate(packet.measurements):
            native = (packet.start_deg + step * i) % 360.0
            if self._last_angle is not None:
                self._swept += (native - self._last_angle) % 360.0
            self._last_angle = native

            if self._swept >= 360.0 and self._points:
                done = self._points
                self._points = []
                self._swept = 0.0

            if dist_mm > 0:
                local = (360.0 - native) % 360.0 if self._native_cw else native
                self._points.append(Point(local, dist_mm / 1000.0, intensity))
        return done


# ---------------------------------------------------------------------------
# The device itself
# ---------------------------------------------------------------------------


class LD19:
    """One LD19 on one serial port, read continuously on a daemon thread.

    read_scan() blocks only until the FIRST complete revolution exists
    (about 100 ms after open at the 10 Hz nominal rate); after that it
    always returns the freshest complete revolution immediately, so calling
    it from a faster control loop never stalls the loop and never yields
    None. stats() exposes health counters for the bring-up scripts.
    """

    def __init__(self, port, baud=230400):
        if serial is None:
            raise RuntimeError(
                "pyserial not installed -- pip install pyserial")
        self._ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        self.port = port
        self._parser = PacketParser()
        self._assembler = RevolutionAssembler()
        self._lock = threading.Condition()
        self._latest = None
        self._rev_count = 0
        self._speed_dps = 0.0
        self._pts_last_rev = 0
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True,
                                        name=f"ld19:{port}")
        self._thread.start()

    # -- reader thread ---------------------------------------------------
    def _reader(self):
        while self._running:
            try:
                # A concurrent close() can null out the port's internals
                # between this check and the read; the guard makes the
                # common case clean.
                if self._ser is None or not self._ser.is_open:
                    break
                data = self._ser.read(PACKET_LEN * 4)
            except BaseException:
                # Once we are shutting down, ANY exception from the port
                # (TypeError from the size-goes-None race, OSError on a
                # yanked USB cable, a KeyboardInterrupt delivered to this
                # thread, etc.) is expected and must never surface as a
                # thread traceback. While still running, a transient error
                # is worth a brief backoff and retry; a persistent one will
                # trip the is_open guard above next iteration.
                if not self._running:
                    break
                time.sleep(0.1)
                continue
            if not data:
                continue
            for pkt in self._parser.feed(data):
                self._speed_dps = pkt.speed_dps
                rev = self._assembler.feed(pkt)
                if rev is not None:
                    with self._lock:
                        self._latest = rev
                        self._rev_count += 1
                        self._pts_last_rev = len(rev)
                        self._lock.notify_all()

    # -- public API ------------------------------------------------------
    def read_scan(self, timeout=2.0):
        """Freshest complete revolution as list[Point]; blocks only before
        the first revolution has been assembled."""
        with self._lock:
            if self._latest is None:
                self._lock.wait_for(lambda: self._latest is not None,
                                    timeout=timeout)
            if self._latest is None:
                raise TimeoutError(
                    f"no LiDAR revolution from {self.port} within "
                    f"{timeout}s -- is it powered and is this the right "
                    "port? (scripts/lidar_test.py --list)")
            return self._latest

    def stats(self):
        with self._lock:
            return {
                "port": self.port,
                "revolutions": self._rev_count,
                "points_last_rev": self._pts_last_rev,
                "speed_dps": self._speed_dps,
                "scan_hz": self._speed_dps / 360.0,
                "crc_errors": self._parser.crc_errors,
                "resyncs": self._parser.resyncs,
            }

    def close(self):
        # Order matters: signal the reader to stop and let it exit its
        # current blocking read() FIRST, then close the port. Closing while
        # the reader is still inside read() is the race that produced the
        # 'NoneType ... integer' traceback. cancel_read() (pyserial >= 3.1)
        # unblocks a read already in flight so the join is quick; if it is
        # unavailable, the port's 0.2s read timeout bounds the wait anyway.
        self._running = False
        try:
            self._ser.cancel_read()
        except Exception:
            pass
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------


def find_candidate_ports():
    """CP210x serial ports that could be LD19 adapters, preferring the
    stable /dev/serial/by-path names.

    Both of this robot's CP2102 adapters report the SAME USB serial number
    ("0001"), so by-id symlinks collide and ttyUSBn ordering shuffles
    between boots. by-path encodes the physical hub socket, which only
    changes if a plug is moved.
    """
    import glob
    import os

    by_path = sorted(glob.glob("/dev/serial/by-path/*"))
    if by_path:
        out = []
        for link in by_path:
            target = os.path.realpath(link)
            if list_ports is not None:
                for p in list_ports.comports():
                    if p.device == target and p.vid == 0x10C4 \
                            and p.pid == 0xEA60:
                        out.append(link)
                        break
            else:
                out.append(link)
        if out:
            return out
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()
            if p.vid == 0x10C4 and p.pid == 0xEA60]
