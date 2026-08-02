/**
 * @file polarity_test.ino
 * @brief Standalone diagnostic that cycles through every individual and
 * combined left/right motor polarity, holding each one for a fixed duration,
 * so the correct wiring polarity for each side can be determined by direct
 * observation instead of inferred from indirect behavior.
 * @date 2026-07-16
 */

// Pin numbers match motor_test.ino. This sketch drives set_side() directly
// with raw signed values, without any per-channel sign correction, so the
// true per-channel polarity can be read off cleanly. Does not use Bridge or
// Serial1.
//
// Channel A is physically the RIGHT side and channel B is physically the
// LEFT side, confirmed by a prior run of this sketch (see motor_test.ino's
// pin comments for the full writeup).
//
// A prior version of this sketch used a two-direction-pin-per-channel
// TB6612 wiring assumption (AIN1/AIN2, BIN1/BIN2). That was wrong for this
// ELEGOO shield: it exposes only one direction pin per channel to the MCU
// (AIN1 for channel A, BIN1 for channel B); the TB6612's other direction
// input is hardwired on the shield PCB, confirmed against ELEGOO's own
// DeviceDriverSet_xxx0.h/.cpp reference code for this shield. The old "AIN2"
// pin (8) was actually the real BIN1, and the old "BIN1"/"BIN2" pins (9, 11)
// were the IR receiver and Y-axis servo signal, unrelated to the driver.
// That bug fully explains the earlier symptom where the left channel's
// observed direction tracked the right channel's commanded sign instead of
// its own: writing the old "AIN2" was silently driving the real left
// channel's only direction pin. It was not a hardware fault.

// Physically RIGHT motor pair (TB6612 channel A). Confirmed working
// correctly: positive value -> AIN1 HIGH -> physically forward.
const int PWMA = 5;
const int AIN1 = 7;

// Physically LEFT motor pair (TB6612 channel B). BIN1 is pin 8, not 9; the
// prior pin 9/11 assignment never reached the driver. Same HIGH=forward
// convention as channel A, inferred from incidental data while pin 8 was
// misused as "AIN2"; re-run this sketch to confirm directly now that BIN1 is
// wired to the correct pin.
const int PWMB = 6;
const int BIN1 = 8;

const int STBY = 3;

const int TEST_PWM = 150;            // PWM magnitude to hold, 0-255
const unsigned long HOLD_MS = 15000; // how long each labeled state is held
const unsigned long STOP_MS = 3000;  // pause between states

/**
 * @brief Configures the TB6612 pins and enables the driver.
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
}

/**
 * @brief Drives one TB6612 channel at a raw signed PWM value, with no
 * per-channel polarity correction applied.
 *
 * @param pwm_pin PWM pin for this channel.
 * @param dir_pin Direction pin for this channel.
 * @param value Signed PWM magnitude in [-255, 255]. Positive sets dir_pin
 * HIGH; negative sets it LOW. Zero stops that channel.
 */
void set_side(int pwm_pin, int dir_pin, int value)
{
    bool positive = value >= 0;
    digitalWrite(dir_pin, positive ? HIGH : LOW);
    int magnitude = abs(value);
    if (magnitude > 255)
    {
        magnitude = 255;
    }
    analogWrite(pwm_pin, magnitude);
}

/**
 * @brief Stops both channels.
 */
void stop_both()
{
    set_side(PWMA, AIN1, 0);
    set_side(PWMB, BIN1, 0);
}

/**
 * @brief Holds one named raw polarity combination for HOLD_MS, with a short
 * stop beforehand so each state is visually distinct from the previous one.
 *
 * left_value and right_value are physical sides, not TB6612 channel letters:
 * left_value is routed to channel B (physically left) and right_value to
 * channel A (physically right), per the confirmed mapping in the pin
 * comments above.
 *
 * @param label Name printed to Serial to identify this state.
 * @param left_value Raw signed value sent to the physically left channel B.
 * @param right_value Raw signed value sent to the physically right channel A.
 */
void hold_state(const char *label, int left_value, int right_value)
{
    stop_both();
    delay(STOP_MS);
    Serial.println("\n\n");
    Serial.println(label);

    set_side(PWMB, BIN1, left_value);
    set_side(PWMA, AIN1, right_value);
    delay(HOLD_MS);
}

/**
 * @brief Cycles through every individual-side and combined polarity forever:
 * left alone in each direction, right alone in each direction, then all four
 * same-sign and opposite-sign combinations of both together.
 */
void loop()
{
    Serial.println("STARTING\n\n");

    hold_state("LEFT ONLY, positive", TEST_PWM, 0);
    hold_state("LEFT ONLY, negative", -TEST_PWM, 0);
    hold_state("RIGHT ONLY, positive", 0, TEST_PWM);
    hold_state("RIGHT ONLY, negative", 0, -TEST_PWM);
    hold_state("BOTH, left+ right+", TEST_PWM, TEST_PWM);
    hold_state("BOTH, left- right-", -TEST_PWM, -TEST_PWM);
    hold_state("BOTH, left+ right-", TEST_PWM, -TEST_PWM);
    hold_state("BOTH, left- right+", -TEST_PWM, TEST_PWM);

    stop_both();
    delay(STOP_MS);
    exit(0);
}
