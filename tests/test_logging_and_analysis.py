"""Run logging and the time-to-capture analysis built on top of it."""

import math

import pytest

from app import config
from app.utils.logging import RunLogger
from scripts import analyze_runs


OPEN = {name: math.inf for name in config.SECTORS}


def _write_run(path, rows, caught_at=None):
    """Write a run CSV with `rows` frames, optionally caught at a frame index."""
    logger = RunLogger(str(path))
    for i in range(rows):
        caught = caught_at is not None and i >= caught_at
        logger.log(fps=12.0, bearing=0.1, proximity=0.5, caught=caught,
                   sectors=OPEN, left_pwm=200, right_pwm=180,
                   identity_name="charlie")
    logger.close()
    return str(path)


def test_logger_writes_header_and_rows(tmp_path):
    path = _write_run(tmp_path / "run.csv", 3)
    lines = open(path).read().strip().splitlines()
    assert lines[0].split(",") == RunLogger.FIELDS
    assert len(lines) == 4                      # header + 3 rows


def test_logger_records_identity_column(tmp_path):
    path = _write_run(tmp_path / "run.csv", 2)
    assert "charlie" in open(path).read()


def test_infinite_sectors_written_as_blank(tmp_path):
    """inf would break downstream float parsing; it is stored as empty."""
    path = _write_run(tmp_path / "run.csv", 1)
    row = open(path).read().strip().splitlines()[1]
    assert "inf" not in row


def _write_csv(path, rows):
    """Write a run CSV with explicit (t, caught) rows, bypassing wall clock."""
    with open(path, "w", newline="") as fh:
        fh.write(",".join(RunLogger.FIELDS) + "\n")
        for t, caught in rows:
            fh.write(f"{t},10.0,0.0,0.5,{int(caught)},charlie,"
                     f",,,,,,100,100\n")
    return str(path)


def test_survival_time_is_exactly_the_first_caught_timestamp(tmp_path):
    """Caught at t=3.0 and still caught after: survival is 3.0, not 5.0."""
    path = _write_csv(tmp_path / "caught.csv",
                      [(0.0, False), (1.0, False), (2.0, False),
                       (3.0, True), (4.0, True), (5.0, True)])
    stats = analyze_runs.summarize(path)
    assert not stats["escaped"]
    assert stats["survival"] == pytest.approx(3.0)
    assert stats["duration"] == pytest.approx(5.0)


def test_escaped_run_survival_equals_full_duration(tmp_path):
    path = _write_csv(tmp_path / "escaped.csv",
                      [(0.0, False), (2.5, False), (7.5, False)])
    stats = analyze_runs.summarize(path)
    assert stats["escaped"]
    assert stats["survival"] == pytest.approx(7.5)


def test_survival_time_is_first_caught_timestamp(tmp_path):
    path = _write_run(tmp_path / "caught.csv", 10, caught_at=4)
    stats = analyze_runs.summarize(path)
    assert not stats["escaped"]
    assert stats["survival"] <= stats["duration"]


def test_escaped_run_survives_the_full_duration(tmp_path):
    path = _write_run(tmp_path / "escaped.csv", 10)
    stats = analyze_runs.summarize(path)
    assert stats["escaped"]
    assert stats["survival"] == stats["duration"]


def test_summary_reports_fps_and_identity(tmp_path):
    path = _write_run(tmp_path / "run.csv", 5)
    stats = analyze_runs.summarize(path)
    assert stats["fps_mean"] == pytest.approx(12.0)
    assert stats["identities"] == ["charlie"]
    assert stats["rows"] == 5


def test_empty_log_is_handled(tmp_path):
    path = tmp_path / "empty.csv"
    RunLogger(str(path)).close()               # header only
    assert analyze_runs.summarize(str(path)) is None


def test_cornered_fraction_counts_converging_walls(tmp_path):
    close = config.CORNER_DISTANCE / 2
    logger = RunLogger(str(tmp_path / "corner.csv"))
    for i in range(10):
        sectors = dict(OPEN)
        if i < 5:                               # half the run boxed in
            sectors["front_left"] = close
            sectors["front_right"] = close
        logger.log(10.0, 0.0, 0.5, False, sectors, 100, 100)
    logger.close()
    stats = analyze_runs.summarize(str(tmp_path / "corner.csv"))
    assert stats["cornered_frac"] == pytest.approx(0.5)
