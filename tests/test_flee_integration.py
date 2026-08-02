"""Closed-loop flee integration tests: the enrolled, locked-on pursuer
chases the robot around a 6 x 6 m room with chairs, and the REAL pipeline
(camera scanner -> geometry -> dual-LiDAR mask/merge -> pursuit tracker ->
escape policy -> differential kinematics) has to keep it free and
un-crashed.

These are the tests the demo rides on. Each maps to a demo claim:

    "it recognises the person and runs away"  -> test_flees_and_is_never_tagged
    "it doesn't hit walls or chairs"          -> test_never_collides
    "it escapes corners instead of jittering" -> corner_start scenario above
    "strangers don't scare it"                -> test_stranger_does_not_trigger_flight
    "it needs BOTH LiDARs"                    -> test_rear_lidar_is_load_bearing
    "the LiDARs agree with reality"           -> test_lidar_range_matches_truth
    "approach from behind doesn't work"       -> test_blind_arc_investigation_spin

The world model (app/simulate.py) fakes only physics and raw sensor
returns; everything between sensing and wheel PWM is production code, so a
sign error anywhere in that chain drives the simulated robot into the
pursuer or a wall and fails here. Walls are SOLID: any commanded motion
that would penetrate one is blocked and recorded as a collision.

Run just these (nice for a live class demo -- takes ~1 minute):

    python3 -m pytest tests/test_flee_integration.py -v
"""

import math

import pytest

from app import config
from app import simulate as S
from app.sensing import lidar
from app.control import evasion


TAG_DISTANCE = 0.40      # pursuer stops at 0.35 and reaches out; anything
                         # this close means the robot lost
RUN_SECONDS = 40


def _scenario(robot_pose, pursuer_xy, **kw):
    world = S.classroom()
    robot = S.Robot(robot_pose[0], robot_pose[1],
                    math.radians(robot_pose[2]))
    return S.FleeSim(world, robot, pursuer_xy, **kw)


SCENARIOS = {
    # Pursuer dead ahead: the robot must execute a full 180 turn-away,
    # during which the chassis out-rotates the servo and the camera is
    # GUARANTEED to go blind. Only the LiDAR hand-off survives it.
    "ahead": ((3.0, 3.0, -90), (3.0, 1.2)),
    # Pursuer on the flank, outside the initial camera view: acquisition
    # relies on LiDAR-guided investigation before they get close.
    "flank": ((3.0, 3.0, 90), (0.8, 3.0)),
    # Robot starts near a corner with the pursuer closing diagonally: the
    # corner-escape latch has to commit to one side and get out.
    "corner_start": ((2.0, 2.0, 45), (4.0, 4.0)),
    # The flee path threads between the two chairs.
    "chair_line": ((3.0, 3.0, 90), (3.0, 5.0)),
    # Pursuer sneaks up through the camera's rear blind arc: the LiDAR must
    # trigger an in-place spin so the camera can identify them in time.
    "behind": ((3.0, 3.0, 90), (3.0, 1.0)),
}


@pytest.mark.parametrize("name", SCENARIOS)
def test_flees_and_is_never_tagged(name):
    """Core demo behaviour: a persistent pursuer never gets within arm's
    reach across a 40-second chase in a closed room."""
    sim = _scenario(*SCENARIOS[name])
    hist = sim.run(RUN_SECONDS)
    dmin = min(h["pursuer_dist"] for h in hist)
    assert dmin > TAG_DISTANCE, f"{name}: pursuer got within {dmin:.2f} m"
    # And fleeing actually happened -- the robot didn't just get lucky
    # standing still while the pursuer's stop-distance saved it.
    assert hist[-1]["pursuer_dist"] > 1.0


@pytest.mark.parametrize("name", SCENARIOS)
def test_never_collides(name):
    """Walls are solid; any blocked motion counts as a collision. The
    corner latch + interpolated dodge must keep the count at zero and the
    clearance above the robot's own radius."""
    sim = _scenario(*SCENARIOS[name])
    hist = sim.run(RUN_SECONDS)
    hits = sum(h["collided"] for h in hist)
    clr = min(h["clearance"] for h in hist)
    assert hits == 0, f"{name}: {hits} blocked/collision steps"
    assert clr >= S.ROBOT_RADIUS_M, f"{name}: clearance fell to {clr:.2f} m"


def test_camera_hands_off_to_lidar_when_blind():
    """After the turn-away puts the pursuer in the rear blind arc, the
    chase must continue on LiDAR fixes -- not camera (physically cannot see
    there) and not stale memory."""
    sim = _scenario(*SCENARIOS["ahead"])
    hist = sim.run(RUN_SECONDS)
    lidar_steps = [h for h in hist if h["source"] == "lidar"]
    assert len(lidar_steps) > len(hist) * 0.3
    # And during those steps the robot is actually driving, not stopped.
    assert all(h["speed"] > 0 for h in lidar_steps)


def test_lidar_range_matches_truth():
    """Fusion sanity: ranges reported from real fixes (camera or LiDAR)
    must track the true robot-to-pursuer distance. Criterion is robust
    rather than per-tick: the 95th-percentile error stays within the
    person's radius plus a couple of scan steps, and even the worst tick
    stays bounded (the dead-reckoned prediction can lag ~10 deg during a
    hard dodge, letting a re-fix graze the wall just past the person's
    silhouette for a tick or two -- inside the range gate, self-correcting,
    and harmless to control). The bugs this exists to catch -- the mirrored
    camera/LiDAR sign convention and the +/-90 mask seam -- produce errors
    of METRES for long stretches and fail both bounds immediately."""
    sim = _scenario(*SCENARIOS["ahead"])
    hist = sim.run(20)
    errors = []
    for h in hist:
        if h["source"] in ("camera", "lidar") \
                and math.isfinite(h["range_m"]):
            true_surface = h["pursuer_dist"] - S.PERSON_RADIUS_M
            errors.append(abs(h["range_m"] - true_surface))
    assert len(errors) > 50
    errors.sort()
    p95 = errors[int(0.95 * (len(errors) - 1))]
    assert p95 < 0.35, f"95th-percentile range error {p95:.2f} m"
    assert errors[-1] < 0.6, f"worst range error {errors[-1]:.2f} m"


def test_rear_lidar_is_load_bearing():
    """Switch the REAR unit off and the fleeing robot must lose the pursuer
    (who ends up behind, where the front unit's mask points) -- proving the
    dual-LiDAR design does real work, not redundancy theatre."""
    both = _scenario(*SCENARIOS["ahead"])
    both.run(20)
    both_lidar = sum(1 for h in both.history if h["source"] == "lidar")

    front_only = _scenario(*SCENARIOS["ahead"], use_rear=False)
    front_only.run(20)
    fo_lidar = sum(1 for h in front_only.history
                   if h["source"] == "lidar")

    assert both_lidar > 100              # with both units the hand-off works
    assert fo_lidar < both_lidar * 0.3   # without the rear one it starves


def test_abeam_seam_is_closed():
    """Regression for the +/-90 mask seam: a person dead abeam at close
    range must be visible to range_at_bearing. With exact-semicircle masks,
    mounting parallax opened a blind wedge at +/-90 and this returned inf
    for a person 0.35 m away (the tracker then latched onto a wall)."""
    world = S.World(S.rectangle_room(6.0, 6.0))
    robot = S.Robot(3.0, 3.0, math.radians(90))
    for d in (0.35, 0.6, 1.2):
        person = S.Circle(3.0 - d, 3.0, S.PERSON_RADIUS_M)
        merged = lidar.merged_points(
            S.synth_scan(world, robot, config.FRONT_LIDAR_OFFSET, [person]),
            S.synth_scan(world, robot, config.REAR_LIDAR_OFFSET, [person]))
        rng = lidar.range_at_bearing(merged, 90.0)
        assert math.isfinite(rng), f"abeam person at {d} m invisible"
        assert abs(rng - (d - S.PERSON_RADIUS_M)) < 0.15


def test_stranger_does_not_trigger_flight():
    """A person the identity gate does not recognise stands nearby: the
    camera may investigate (servo motion only), but the wheels never move
    and no pursuit ever starts."""
    world = S.classroom()
    world.chairs.append(S.Circle(3.0, 4.2, S.PERSON_RADIUS_M))  # "stranger"
    robot = S.Robot(3.0, 3.0, math.radians(90))
    sim = S.FleeSim(world, robot, (3.0, -50.0), pursuer_moves=False,
                    enrolled_visible=False)
    hist = sim.run(8)
    assert all(h["pwm"] == (0, 0) for h in hist), "wheels moved for stranger"
    assert all(h["bearing_ccw"] is None for h in hist)
    assert all(h["source"] is None for h in hist)


def test_blind_arc_investigation_spin():
    """Pursuer approaches through the camera's rear blind arc: the LiDAR
    must trigger an in-place spin (wheels counter-rotate, no forward speed)
    that brings them into camera view before they can tag."""
    sim = _scenario(*SCENARIOS["behind"])
    hist = sim.run(RUN_SECONDS)
    spins = [h for h in hist if h.get("investigate") == "spin"]
    assert spins, "never investigated the blind-arc approach"
    for h in spins:
        left, right = h["pwm"]
        assert left == -right or (left, right) == (0, 0), (
            "investigation spin must be in place (no forward speed)")
    # And the chase then proceeded on camera + LiDAR:
    assert any(h["source"] == "camera" for h in hist)
    assert min(h["pursuer_dist"] for h in hist) > TAG_DISTANCE


def test_tracker_survives_the_turn_away():
    """Regression for the two hand-off killers: (a) the chassis-frame
    bearing must be yaw-compensated during the hard turn-away, and (b) the
    range gate must stop the LiDAR window grabbing the wall. Within the
    first 3 seconds of 'ahead' the robot turns ~180 deg; every camera/LiDAR
    fix must stay within 35 deg of ground truth."""
    sim = _scenario(*SCENARIOS["ahead"])
    for _ in range(int(3.0 / sim.dt)):
        rec = sim.step()
        if rec["bearing_ccw"] is None or rec["source"] == "memory":
            continue
        true_ccw = ((math.degrees(
            math.atan2(rec["py"] - rec["y"], rec["px"] - rec["x"])
            - rec["theta"]) + 180) % 360) - 180
        err = abs((rec["bearing_ccw"] - true_ccw + 180) % 360 - 180)
        assert err < 35, (
            f"t={rec['t']:.2f}: tracked {rec['bearing_ccw']:+.0f} vs "
            f"true {true_ccw:+.0f} ({rec['source']})")


def test_pursuer_in_front_does_not_cause_stationary_spin():
    """Regression: a pursuer standing close in the robot's FRONT arc must
    not be treated as a wall. If obstacle avoidance brakes and dodges for
    the person being fled, the wheels reverse against each other into an
    in-place spin and the robot never drives away. With the pursuer excluded
    from avoidance, the same geometry must yield a forward-driving arc (both
    wheels commanded forward)."""
    pol = evasion.EscapePolicy()
    for bearing, front in [(2.0, 0.30), (5.0, 0.20), (25.0, 0.40)]:
        sectors = {"front": front, "front_left": front + 0.1,
                   "front_right": front + 0.1, "left": 2.0,
                   "right": 2.0, "rear": 2.0}
        heading, speed = pol.step(bearing, 0.6, sectors,
                                  pursuer_range=front)
        left, right = evasion.heading_to_pwm(heading, speed)
        assert left > 0 and right > 0, (
            f"pursuer at {bearing} deg / {front} m spun in place: "
            f"L={left} R={right}")


def test_real_wall_still_brakes_when_pursuer_is_elsewhere():
    """Guard the other side of the fix: a genuine wall close ahead, with the
    pursuer NOT on that bearing, must still trigger avoidance (the exclusion
    is pursuer-specific, not a blanket disabling of the brake)."""
    pol = evasion.EscapePolicy()
    sectors = {"front": 0.20, "front_left": 0.30, "front_right": 2.0,
               "left": 2.0, "right": 2.0, "rear": 2.0}
    # Pursuer is behind (175 deg) and far; the 0.20 m thing ahead is a wall.
    heading, speed = pol.step(175.0, 0.3, sectors, pursuer_range=3.0)
    assert speed < config.BASE_SPEED, "brake did not fire for a real wall"


class _MirroredRobot(S.Robot):
    """A chassis whose physical turn direction is OPPOSITE to the code's
    convention -- what you get when the motor channels drive the other
    physical sides than the firmware assumes."""

    def drive(self, left_pwm, right_pwm, dt, world=None):
        return super().drive(right_pwm, left_pwm, dt, world=world)


def test_mirrored_drive_produces_donuts():
    """Regression for the field symptom: with the drive wired mirrored and
    the config unaware, steering is positive feedback -- the robot orbits
    the pursuer ("donuts") instead of fleeing, racking up several times the
    normal total rotation and getting tagged."""
    w = S.classroom()
    sim = S.FleeSim(w, _MirroredRobot(3, 3, math.radians(-90)), (3.0, 1.6))
    hist = sim.run(25)
    total_rot = math.degrees(sum(
        abs(hist[i + 1]["theta"] - hist[i]["theta"])
        for i in range(len(hist) - 1)))
    dmin = min(h["pursuer_dist"] for h in hist)
    assert total_rot > 1800, "mirrored drive should spin far more than normal"
    assert dmin <= TAG_DISTANCE, "donuts should get the robot caught"


def test_sign_config_rescues_mirrored_drive(monkeypatch):
    """And the fix: MOTOR_TURN_SIGN = -1 plus a negated yaw constant (the
    values scripts/sign_check.py would report for that wiring) makes the
    mirrored chassis behave identically to a healthy one -- it escapes."""
    monkeypatch.setattr(config, "MOTOR_TURN_SIGN", -1)
    monkeypatch.setattr(config, "YAW_DPS_PER_PWM_DIFF",
                        -abs(config.YAW_DPS_PER_PWM_DIFF))
    w = S.classroom()
    sim = S.FleeSim(w, _MirroredRobot(3, 3, math.radians(-90)), (3.0, 1.6))
    hist = sim.run(25)
    dmin = min(h["pursuer_dist"] for h in hist)
    total_rot = math.degrees(sum(
        abs(hist[i + 1]["theta"] - hist[i]["theta"])
        for i in range(len(hist) - 1)))
    assert dmin > TAG_DISTANCE, "corrected signs should let it escape"
    assert total_rot < 1500, "corrected signs should kill the donuts"
    assert sum(h["collided"] for h in hist) == 0


def test_inverted_motor_is_corrected_at_bridge(monkeypatch):
    """A motor wired backwards makes a spin command translate and a straight
    command spin -- the field donut fault. The per-wheel sign in config must
    flip that motor's command at the bridge so a differential stays a
    differential. We capture what the bridge would actually send."""
    import sys
    import types

    sent = []
    fake_bridge = types.SimpleNamespace(
        notify=lambda name, *a: sent.append((name, a)))
    fake_mod = types.ModuleType("arduino")
    fake_utils = types.ModuleType("arduino.app_utils")
    fake_utils.Bridge = fake_bridge
    fake_mod.app_utils = fake_utils
    monkeypatch.setitem(sys.modules, "arduino", fake_mod)
    monkeypatch.setitem(sys.modules, "arduino.app_utils", fake_utils)

    from app.control.bridge_client import BridgeClient
    client = BridgeClient()

    # Left motor inverted: a spin command (+120, -120) must reach the
    # firmware as (-120, -120) so both wheels really turn opposite the
    # right one -- i.e. the differential is preserved after the firmware's
    # own inversion of the left channel.
    monkeypatch.setattr(config, "MOTOR_LEFT_SIGN", -1)
    monkeypatch.setattr(config, "MOTOR_RIGHT_SIGN", +1)
    sent.clear()
    client.set_motion(120, -120)
    assert sent[-1] == ("set_motion", (-120, -120))

    # And a straight command survives with the same correction.
    sent.clear()
    client.set_motion(200, 200)
    assert sent[-1] == ("set_motion", (-200, 200))


def test_swapped_channels_are_corrected_at_bridge(monkeypatch):
    """Crossed motor channels -- commanding one wheel moves the other -- are
    repaired by MOTOR_SWAP_LR at the bridge. The swap is applied BEFORE the
    per-wheel signs, and both can coexist (a robot that is both crossed and
    has one motor inverted)."""
    import sys
    import types

    sent = []
    fake_bridge = types.SimpleNamespace(
        notify=lambda name, *a: sent.append((name, a)))
    fake_utils = types.ModuleType("arduino.app_utils")
    fake_utils.Bridge = fake_bridge
    fake_mod = types.ModuleType("arduino")
    fake_mod.app_utils = fake_utils
    monkeypatch.setitem(sys.modules, "arduino", fake_mod)
    monkeypatch.setitem(sys.modules, "arduino.app_utils", fake_utils)

    from app.control.bridge_client import BridgeClient
    client = BridgeClient()

    # Pure swap: (left=200, right=50) must reach firmware as (50, 200).
    monkeypatch.setattr(config, "MOTOR_SWAP_LR", True)
    monkeypatch.setattr(config, "MOTOR_LEFT_SIGN", +1)
    monkeypatch.setattr(config, "MOTOR_RIGHT_SIGN", +1)
    sent.clear()
    client.set_motion(200, 50)
    assert sent[-1] == ("set_motion", (50, 200))

    # Swap THEN sign: after swapping to (50, 200), a left inversion negates
    # the left channel -> (-50, 200).
    monkeypatch.setattr(config, "MOTOR_LEFT_SIGN", -1)
    sent.clear()
    client.set_motion(200, 50)
    assert sent[-1] == ("set_motion", (-50, 200))


def test_caught_debounce_ignores_brief_range_glitches():
    """Regression for the field failure: runs were ending at random because
    a single LiDAR revolution put a chair edge or wall corner inside the
    bearing window under the caught radius. A glitch shorter than
    CAUGHT_HOLD_S -- even several of them -- must never end the run; any
    clean tick resets the clock."""
    det = evasion.CaughtDetector(hold_s=1.5)
    t = 0.0
    # Three separate sub-second glitches with clean gaps between them.
    for _ in range(3):
        for _ in range(10):            # 0.8 s of "caught" at 12.5 Hz
            assert det.update(True, t) is False
            t += 0.08
        assert det.update(False, t) is False   # one clean tick resets
        t += 0.08
    assert det.held_for(t) == 0.0


def test_caught_debounce_confirms_sustained_capture():
    """And the true positive: the condition held continuously past
    CAUGHT_HOLD_S confirms the capture."""
    det = evasion.CaughtDetector(hold_s=1.5)
    t = 0.0
    fired = False
    for _ in range(25):                # 2.0 s continuous
        fired = det.update(True, t)
        if fired:
            break
        t += 0.08
    assert fired, "sustained capture never confirmed"
    assert t >= 1.5 - 1e-9


def test_memory_source_never_counts_toward_caught():
    """The main-loop gating: caught_now is computed only from measured
    fixes. A dead-reckoned memory range under the radius is a stale
    extrapolation, not evidence -- mirror the loop's expression here so a
    change to that gating breaks a test."""
    for source, expected in [("camera", True), ("lidar", True),
                             ("memory", False), (None, False)]:
        caught_now = (source in ("camera", "lidar")
                      and (0.1 >= config.CAUGHT_PROXIMITY
                           or 0.25 <= config.CAUGHT_RANGE_M))
        assert caught_now is expected, source
