/**
 * @file motor_test.ino
 * @brief Standalone MCU-only sketch to sanity-check TB6612 motor wiring and
 * direction on the UNO Q.
 * @date 2026-07-14
 */

// Pin numbers below come from the stock ELEGOO Smart Robot Car V4.0 wiring for
// the original UNO R3 and are confirmed correct for the UNO Q by direct testing
// with polarity_test.ino. That testing also found the physical side each
// channel drives is the opposite of what the TB6612's own channel naming would
// suggest: channel A is physically the RIGHT side and channel B is physically
// the LEFT side.
//
// This shield exposes only one direction pin per channel to the MCU (AIN1 for
// channel A, BIN1 for channel B); the TB6612's other direction input is
// hardwired on the shield PCB. This was confirmed against ELEGOO's own
// DeviceDriverSet_xxx0.h/.cpp reference code for this shield, which defines
// only PIN_Motor_AIN_1 and PIN_Motor_BIN_1 (no AIN2/BIN2). An earlier version
// of this sketch used a two-direction-pin-per-channel assumption with BIN1 on
// pin 9; that pin never reached the driver (it is the IR receiver line), and
// the real BIN1 (pin 8) was being driven incidentally by this sketch's old
// "AIN2" writes, which is why the left channel appeared to track the right
// channel's commanded sign instead of its own. Both channels use the same
// polarity convention: positive value -> dir pin HIGH -> physically forward,
// confirmed for channel A directly and for channel B from that incidental
// data; re-run polarity_test.ino after this pin fix to confirm channel B
// directly. This sketch does not use the Bridge, Serial1, or any
// router-reserved resource, so it is safe to flash and run standalone.

// Physically RIGHT motor pair (TB6612 channel A). Confirmed working correctly.
const int PWMA = 5;
const int AIN1 = 7;

// Physically LEFT motor pair (TB6612 channel B). BIN1 is pin 8.
const int PWMB = 6;
const int BIN1 = 8;

// TB6612 standby, active HIGH
const int STBY = 3;

// Test tuning constants
const int TEST_PWM = 150;            // PWM magnitude for every move, 0-255
const unsigned long MOVE_MS = 1000;  // duration of each straight leg
const unsigned long PAUSE_MS = 5000; // pause between maneuvers
const unsigned long PIVOT_MS =
    500; // duration of each ~90 degree pivot; UNVERIFIED, tune by observation

/**
 * @brief Configures the TB6612 direction and PWM pins, enables the driver, and
 * runs the bring-up sequence once.
 */
void setup()
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    Serial.begin(9600);
    delay(5000);
    run_test_sequence();
}

/**
 * @brief Idles the motors. The test sequence runs once from setup(); loop()
 * only guarantees the robot stays stopped afterward.
 */
void loop()
{
    stop_motors();
}

/**
 * @brief Drives one TB6612 channel at a signed PWM value.
 *
 * @param pwm_pin PWM pin for this channel.
 * @param dir_pin Direction pin for this channel.
 * @param value Signed PWM magnitude in [-255, 255]. Positive is forward and
 * negative is reverse, confirmed for both channels by polarity_test.ino.
 */
void set_side(int pwm_pin, int dir_pin, int value)
{
    bool forward = value >= 0;
    digitalWrite(dir_pin, forward ? HIGH : LOW);
    int magnitude = abs(value);
    if (magnitude > 255)
    {
        magnitude = 255;
    }
    analogWrite(pwm_pin, magnitude);
}

/**
 * @brief Drives both wheels at independent signed PWM values.
 *
 * left_pwm and right_pwm are physical sides, not TB6612 channel letters:
 * left_pwm is routed to channel B and right_pwm to channel A, per the
 * mapping confirmed in polarity_test.ino (see the pin comments above). Both
 * channels use the same positive-is-forward convention.
 *
 * @param left_pwm Signed PWM for the physically left wheel pair, in
 * [-255, 255]. Routed to channel B.
 * @param right_pwm Signed PWM for the physically right wheel pair, in
 * [-255, 255]. Routed to channel A.
 */
void drive(int left_pwm, int right_pwm)
{
    set_side(PWMB, BIN1, left_pwm);
    set_side(PWMA, AIN1, right_pwm);
}

/**
 * @brief Stops both wheels.
 */
void stop_motors()
{
    drive(0, 0);
}

/**
 * @brief Runs the fixed bring-up maneuver once: forward, pivot right, forward,
 * pivot left, then halt. Logs each step over USB Serial so it can be matched to
 * what the robot physically does.
 *
 * Pivot right is left-wheel-forward, right-wheel-reverse (and pivot left the
 * opposite); this follows from the confirmed per-channel polarity, but the
 * exact pivot angle for PIVOT_MS is still UNVERIFIED.
 */
void run_test_sequence()
{
    Serial.println("forward");
    drive(TEST_PWM, TEST_PWM);
    delay(MOVE_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("pivot right");
    drive(TEST_PWM, -TEST_PWM);
    delay(PIVOT_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("forward");
    drive(TEST_PWM, TEST_PWM);
    delay(MOVE_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("pivot left");
    drive(-TEST_PWM, TEST_PWM);
    delay(PIVOT_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("halt");
    stop_motors();
}
