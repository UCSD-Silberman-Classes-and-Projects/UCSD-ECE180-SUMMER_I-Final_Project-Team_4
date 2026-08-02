/**
 * @file sketch.ino
 * @brief Evasion Bot STM32 side: TB6612 drive motors plus the camera pan
 * servo, exposed to the MPU over the Bridge.
 * @date 2026-07-24
 */

/*
 * Responsibilities (real-time, deterministic work only):
 *   - drive the TB6612 motor channels at commanded PWM
 *   - hold and slew-limit the camera pan servo
 *   - fail safe if the MPU goes quiet
 *
 * The ultrasonic sensor is no longer used: obstacle distance now comes from
 * the two LD19 LiDARs, processed on the MPU (see app/sensing/lidar.py). That
 * removes the old fast local collision reflex, so its job is split:
 *   - speed easing near obstacles happens in the MPU evasion policy
 *   - the LAST-RESORT stop stays here as a command watchdog: if no set_motion
 *     arrives for COMMAND_TIMEOUT_MS the motors halt, so a crashed, wedged or
 *     disconnected MPU can never leave the robot driving. This is the one
 *     safety behaviour that must not depend on Linux being healthy.
 *
 * RPC names must match app/control/bridge_client.py exactly (case-sensitive).
 * Handlers are registered with provide_safe so they run in loop() context
 * rather than the background RPC thread -- they touch analogWrite/Servo.
 */

#include "Arduino_RouterBridge.h"

#include "motors.h"
#include "servo_cam.h"

/** @brief Halt the motors if the MPU has not commanded motion recently. */
const unsigned long COMMAND_TIMEOUT_MS = 500;

/** @brief millis() timestamp of the last set_motion received from the MPU. */
volatile unsigned long last_command_ms = 0;

/** @brief True once the watchdog has halted the motors, cleared on next command. */
volatile bool watchdog_halted = false;

void rpc_set_motion(int left_pwm, int right_pwm);
void rpc_stop();
void rpc_set_servo(int angle_deg);
int rpc_get_servo();

void setup()
{
    motors_begin();
    servo_begin();
    Bridge.begin();
    Bridge.provide_safe("set_motion", rpc_set_motion);
    Bridge.provide_safe("stop", rpc_stop);
    Bridge.provide_safe("set_servo", rpc_set_servo);
    Bridge.provide_safe("get_servo", rpc_get_servo);
    last_command_ms = millis();
}

void loop()
{
    // Slew the pan servo toward its target. Doing this every iteration is
    // what makes the sweep smooth; the MPU only ever posts a destination.
    servo_update();

    // Command watchdog: the MPU sends motion every control iteration, so a
    // gap this long means it is not running. Halt rather than coast.
    if (!watchdog_halted && (millis() - last_command_ms > COMMAND_TIMEOUT_MS))
    {
        watchdog_halted = true;
        motors_stop();
    }

    delay(20);
}

/**
 * @brief RPC handler for "set_motion": commands wheel PWMs from the MPU.
 *
 * @param left_pwm Signed PWM for the physically left wheel pair, [-255, 255].
 * @param right_pwm Signed PWM for the physically right wheel pair, [-255, 255].
 */
void rpc_set_motion(int left_pwm, int right_pwm)
{
    last_command_ms = millis();
    watchdog_halted = false;
    motors_drive(left_pwm, right_pwm);
}

/** @brief RPC handler for "stop": immediately halts both motors. */
void rpc_stop()
{
    last_command_ms = millis();
    motors_stop();
}

/**
 * @brief RPC handler for "set_servo": aim the camera pan servo.
 *
 * Fire-and-forget from the MPU's perspective: the servo slews toward this
 * angle in loop(), so a command sent every control iteration produces a
 * smooth pan rather than a series of jumps.
 *
 * @param angle_deg Target angle in degrees, clamped inside servo_cam.cpp.
 */
void rpc_set_servo(int angle_deg)
{
    servo_set_target(angle_deg);
}

/**
 * @brief RPC handler for "get_servo": current commanded pan angle, degrees.
 *
 * The MPU uses this to convert a bearing measured in the camera frame into a
 * bearing relative to the chassis.
 */
int rpc_get_servo()
{
    return servo_current_deg();
}
