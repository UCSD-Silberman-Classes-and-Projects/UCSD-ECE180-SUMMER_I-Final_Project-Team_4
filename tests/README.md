# Tests

Unit tests for the pure logic -- no camera, board, LiDAR, or model file needed,
so they run on any laptop and in CI.

    pip install pytest
    python -m pytest tests/ -q

| File | Covers |
|---|---|
| `test_geometry.py` | bbox -> bearing / proximity, clamping, monotonicity |
| `test_evasion.py` | flee direction, speed scaling, corner override, PWM mapping |
| `test_lidar.py` | arc math (incl. wraparound), masks, frame transform, sectorizing, corner detection |
| `test_identity.py` | torso signatures, enrollment storage, multi-person pursuer selection |
| `test_logging_and_analysis.py` | run-log format and time-to-capture computation |

Hardware-touching modules (`camera.py`, `detector.py`, `ld19_driver.py`,
`bridge_client.py`) are deliberately NOT unit-tested: they are thin wrappers
whose behavior only means anything against real devices. Verify those with the
bring-up scripts instead -- `benchmark_fps.py`, `lidar_viz.py`,
`stub_smoketest.py`, and the `motor_test` / `polarity_test` sketches.

`test_lidar.py` defines its own `Point` namedtuple rather than importing
`ld19_driver`, which needs the board-only `lds2d` package.
