"""RPC client to the STM32 over the Arduino UNO Q Bridge (arduino.app_utils).

Wraps Bridge so the rest of the app calls set_motion / stop / set_servo and
never sees transport details. This also makes the whole MPU loop testable
with a StubBridge (below) that needs no hardware.

Bridge is a process-wide singleton that connects lazily on first use, so
there is no separate handle to construct or store. The arduino.app_utils
import is deferred into each method, rather than done at module level, so
this module (and StubBridge below) can still be imported on a machine
without that package installed -- e.g. for --stub runs off-board. Motor and
servo commands both use Bridge.notify (fire-and-forget) since they are sent
every control loop iteration and a dropped ack should never stall motion or
the camera pan. Nothing in the loop uses Bridge.call: a blocking round-trip
inside a 15 FPS control loop is exactly the kind of stall we cannot afford,
so the MPU tracks the commanded servo angle itself (see control/scanner.py)
rather than reading it back. get_servo() exists for bring-up scripts only.
"""


from .. import config


class BridgeClient:
    def set_motion(self, left_pwm, right_pwm):
        """Command wheel PWMs (-255..255). Sign sets direction.

        MOTOR_LEFT_SIGN / MOTOR_RIGHT_SIGN absorb a motor whose polarity is
        wired backwards -- the hardware fault where a wheel spins the wrong
        way for a given command. This is the true hardware-adaptation layer
        (it is NOT exercised by the simulator, which drives its own model),
        so a miswired motor is a config change here rather than a firmware
        reflash or a swap of the physical leads. Measured by
        scripts/sign_check.py. With one motor inverted and uncorrected, a
        differential (spin) command translates and a straight command spins
        -- the robot does donuts whenever it tries to flee.
        """
        from arduino.app_utils import Bridge

        if config.MOTOR_SWAP_LR:
            left_pwm, right_pwm = right_pwm, left_pwm
        left_pwm = int(config.MOTOR_LEFT_SIGN * left_pwm)
        right_pwm = int(config.MOTOR_RIGHT_SIGN * right_pwm)
        Bridge.notify("set_motion", left_pwm, right_pwm)

    def stop(self):
        from arduino.app_utils import Bridge

        Bridge.notify("stop")

    def set_servo(self, angle_deg):
        """Aim the camera pan servo (degrees). Firmware slews toward this."""
        from arduino.app_utils import Bridge

        Bridge.notify("set_servo", int(angle_deg))

    def get_servo(self):
        """Read the servo's current commanded angle. Bring-up/testing only --
        this blocks, so do not call it from the control loop."""
        from arduino.app_utils import Bridge

        return Bridge.call("get_servo")


class StubBridge:
    """Hardware-free stand-in: prints commands so the MPU loop can run on a laptop."""

    def set_motion(self, left_pwm, right_pwm):
        print(f"[stub] set_motion(L={left_pwm}, R={right_pwm})")

    def stop(self):
        print("[stub] stop()")

    def set_servo(self, angle_deg):
        print(f"[stub] set_servo({int(angle_deg)})")

    def get_servo(self):
        return config.SERVO_CENTER_DEG
