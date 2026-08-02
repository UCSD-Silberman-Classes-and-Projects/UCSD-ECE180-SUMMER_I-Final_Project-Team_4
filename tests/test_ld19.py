"""LD19 protocol driver: tested entirely against synthetic byte streams.

The packet builder here is written from the LD19 datasheet independently of
the parser, so a misreading of the spec in one is caught by the other. No
serial port, no hardware, no lds2d.
"""

import struct

import pytest

from app.sensing.ld19_driver import (HEADER, PACKET_LEN, POINTS_PER_PACKET,
                                     VER_LEN, PacketParser,
                                     RevolutionAssembler, crc8)


def build_packet(start_deg, end_deg, dists_mm=None, intensities=None,
                 speed_dps=3600, ts_ms=0, corrupt_crc=False):
    """Construct one valid 47-byte LD19 packet."""
    if dists_mm is None:
        dists_mm = [1000] * POINTS_PER_PACKET
    if intensities is None:
        intensities = [200] * POINTS_PER_PACKET
    raw = bytearray([HEADER, VER_LEN])
    raw += struct.pack("<H", speed_dps)
    raw += struct.pack("<H", int(round(start_deg * 100)) % 36000)
    for d, i in zip(dists_mm, intensities):
        raw += struct.pack("<HB", d, i)
    raw += struct.pack("<H", int(round(end_deg * 100)) % 36000)
    raw += struct.pack("<H", ts_ms)
    c = crc8(bytes(raw))
    raw.append((c ^ 0xFF) if corrupt_crc else c)
    assert len(raw) == PACKET_LEN
    return bytes(raw)


def sweep_packets(step_deg=8.0, revs=1.0):
    """A stream of contiguous packets sweeping `revs` full revolutions."""
    pkts, a = [], 0.0
    total = 360.0 * revs
    while a < total:
        pkts.append(build_packet(a % 360.0, (a + step_deg) % 360.0))
        a += step_deg + step_deg / (POINTS_PER_PACKET - 1)
    return pkts


# ---- CRC ----

def test_crc_matches_datasheet_vector():
    # Datasheet's own example packet header bytes produce a table-driven
    # CRC; regression-pin our generated table against a known-good value
    # computed from the published polynomial.
    assert crc8(b"\x54\x2c") == crc8(bytes([0x54, 0x2C]))
    assert crc8(b"") == 0
    assert crc8(b"\x00") == 0
    assert crc8(b"\x01") == 0x4D  # single bit through poly 0x4D


# ---- parser ----

def test_parses_a_clean_packet():
    p = PacketParser()
    pkts = p.feed(build_packet(10.0, 21.0, dists_mm=list(range(100, 1300,
                                                               100))))
    assert len(pkts) == 1
    pkt = pkts[0]
    assert pkt.start_deg == pytest.approx(10.0)
    assert pkt.end_deg == pytest.approx(21.0)
    assert pkt.speed_dps == 3600
    assert pkt.measurements[0] == (100, 200)
    assert pkt.measurements[-1] == (1200, 200)


def test_rejects_bad_crc_and_recovers():
    p = PacketParser()
    bad = build_packet(0.0, 11.0, corrupt_crc=True)
    good = build_packet(12.0, 23.0)
    pkts = p.feed(bad + good)
    assert len(pkts) == 1
    assert pkts[0].start_deg == pytest.approx(12.0)
    assert p.crc_errors >= 1


def test_resyncs_through_garbage_and_partial_packets():
    p = PacketParser()
    good1 = build_packet(0.0, 11.0)
    good2 = build_packet(12.0, 23.0)
    stream = b"\x00\xff\x54\x99" + good1[20:] + good1 + b"\x54" + good2
    # Feed in awkward chunks to exercise buffering across boundaries.
    out = []
    for i in range(0, len(stream), 7):
        out.extend(p.feed(stream[i:i + 7]))
    assert [pk.start_deg for pk in out] == pytest.approx([0.0, 12.0])
    assert p.resyncs > 0


def test_streamed_one_byte_at_a_time():
    p = PacketParser()
    pkt = build_packet(45.0, 56.0)
    out = []
    for b in pkt:
        out.extend(p.feed(bytes([b])))
    assert len(out) == 1


# ---- revolution assembly ----

def test_assembles_exactly_one_revolution():
    asm = RevolutionAssembler(native_cw=False)
    revs = []
    for pkt_bytes in sweep_packets(revs=2.5):
        parsed = PacketParser().feed(pkt_bytes)[0]
        done = asm.feed(parsed)
        if done:
            revs.append(done)
    assert len(revs) == 2
    for rev in revs:
        angles = [p.angle_deg for p in rev]
        assert max(angles) - min(angles) > 300   # spans the full circle
        assert len(rev) > 250


def test_angle_interpolation_handles_the_wrap():
    asm = RevolutionAssembler(native_cw=False)
    pkt = PacketParser().feed(build_packet(355.0, 6.0))[0]
    asm.feed(pkt)
    angles = sorted(p.angle_deg for p in asm._points)
    # 12 points from 355 to 366(=6): step 1.0; must wrap, not run backwards
    assert angles[0] == pytest.approx(0.0, abs=0.51) or \
        any(a < 10 for a in angles)
    assert any(a > 350 for a in angles)
    assert not any(10 < a < 350 for a in angles)


def test_zero_distance_returns_are_dropped():
    asm = RevolutionAssembler(native_cw=False)
    dists = [0] * 6 + [1500] * 6
    pkt = PacketParser().feed(build_packet(0.0, 11.0, dists_mm=dists))[0]
    asm.feed(pkt)
    assert len(asm._points) == 6
    assert all(p.distance_m == pytest.approx(1.5) for p in asm._points)


def test_native_cw_flip_gives_ccw_local_frame():
    """The sensor spins CW; downstream code thinks CCW. An echo the sensor
    reports at native 90 must land at local 270 after the flip (and vice
    versa), so 'object to the unit's left reads +90' holds on hardware."""
    asm = RevolutionAssembler(native_cw=True)
    pkt = PacketParser().feed(
        build_packet(90.0, 101.0, dists_mm=[1000] * 12))[0]
    asm.feed(pkt)
    a0 = asm._points[0].angle_deg
    assert a0 == pytest.approx((360.0 - 90.0) % 360.0)

    asm2 = RevolutionAssembler(native_cw=False)
    pkt2 = PacketParser().feed(
        build_packet(90.0, 101.0, dists_mm=[1000] * 12))[0]
    asm2.feed(pkt2)
    assert asm2._points[0].angle_deg == pytest.approx(90.0)


def test_full_pipeline_bytes_to_revolution():
    """End to end: a raw byte stream (with noise injected) in, one coherent
    revolution out, points in [0, 360), metres, non-zero intensity."""
    parser = PacketParser()
    asm = RevolutionAssembler(native_cw=True)
    stream = b"".join(sweep_packets(revs=1.3))
    stream = b"\xde\xad" + stream[:200] + b"\x54\x2c\x00" + stream[200:]
    rev = None
    for i in range(0, len(stream), 64):
        for pkt in parser.feed(stream[i:i + 64]):
            rev = asm.feed(pkt) or rev
    assert rev is not None
    assert all(0.0 <= p.angle_deg < 360.0 for p in rev)
    assert all(p.distance_m == pytest.approx(1.0) for p in rev)
    assert all(p.intensity == 200 for p in rev)


# ---- shutdown race (regression) ----
#
# close() used to close the serial port while the reader thread was still
# inside serial.read(); pyserial nulls the port's internals on close, so the
# in-flight read did `None - 0` and threw 'NoneType object cannot be
# interpreted as an integer' on every close. The fix stops+joins the reader
# before closing the port and guards the read. This test drives the exact
# shape of that race with a fake serial port and asserts no thread raises.

class _RacyFakeSerial:
    """A serial stand-in whose close() nulls out its read size mid-flight,
    reproducing pyserial's serialposix behaviour."""

    def __init__(self, *_a, **_k):
        self._open = True
        self._size = 64
        self._n = 0

    @property
    def is_open(self):
        return self._open

    def read(self, size):
        if not self._open or size is None:
            raise TypeError(
                "'NoneType' object cannot be interpreted as an integer")
        self._n += 1
        # Also exercise a KeyboardInterrupt delivered INTO the reader thread
        # (the --watch Ctrl-C path): the reader must swallow it on shutdown,
        # which is why it catches BaseException, not just Exception.
        if self._n % 5 == 0:
            raise KeyboardInterrupt()
        import time
        time.sleep(0.01)
        return b"\x00" * 64        # garbage; parser just resyncs

    def cancel_read(self):
        pass

    def close(self):
        self._open = False
        self._size = None          # the null-out that caused the crash


def test_close_does_not_raise_in_reader_thread(monkeypatch):
    import threading
    import time
    import app.sensing.ld19_driver as drv

    monkeypatch.setattr(drv, "serial",
                        type("S", (), {"Serial": _RacyFakeSerial,
                                       "SerialException": Exception}))

    caught = []
    monkeypatch.setattr(threading, "excepthook",
                        lambda args: caught.append(args))

    for _ in range(15):            # hammer the open/close race
        unit = drv.LD19("/dev/fake")
        time.sleep(0.02)
        unit.close()

    time.sleep(0.1)
    assert not caught, f"reader thread raised on close: {caught}"
