#include <Arduino.h>
#include <Keyboard.h>
#include <PWM.h>

#include <VL53L0X.h>
#include <Wire.h>

//#define DEBUG // If you comment this line, the DPRINT & DPRINTLN lines are
              // defined as blank.
#ifdef DEBUG  // Macros are usually in all capital letters.
#define DPRINT(...) Serial.print(__VA_ARGS__) // DPRINT is a macro, debug print
#define DPRINTLN(...)                                                          \
  Serial.println(__VA_ARGS__) // DPRINTLN is a macro, debug print with new line
#else
#define DPRINT(...)   // now defines a blank line
#define DPRINTLN(...) // now defines a blank line
#endif

// Hardware and peripherals
const int led_pin = 10; // LED_BUILTIN is not defined for the micro board

// Wally state variables
unsigned long lockingStamp = 0;
unsigned long wallyLastSeen = 0;
int messageDuration = 0;

const unsigned long WALLY_TOUT = 5000; // milliseconds
const int DISTANCE_THSDL = 1000;       // 1 meter

VL53L0X sensor;
const int Nsamples = 5;
// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY



void setup() {

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial)
    delay(10);
#endif
  Keyboard.begin();
  Wire.begin();

  // --------------------------------------------------------------------------------------------
  // Setup the LED pin PWM

  InitTimersSafe(1);

  // sets the frequency for the specified pin
  int freq = 1; // 1 Hz
  if (!SetPinFrequencySafe(led_pin, freq)) {
    DPRINTLN("Failed to set the frequency for the LED pin!");
  } else {
    DPRINTLN("Frequency of " + String(freq) + " set for the LED pin!");
    pwmWrite(led_pin, 255); // 100% duty cycle, always on
  }

  // --------------------------------------------------------------------------------------------
  // Setup the TOF Sensor
  DPRINTLN("Initializing VL53L0X...");
  sensor.setTimeout(500);
  if (!sensor.init()) {
    DPRINTLN("Failed to detect and initialize sensor!");
    pwmWrite(led_pin, 250); // quick blink -> failure
    while (1)
      ;
  } else {

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    // sensor.startContinuous();

#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
    // increase timing budget to 100 ms
    sensor.setMeasurementTimingBudget(200000);
#endif

    DPRINTLN("Sensor initialized!");
  }

  DPRINTLN("Setup done!");
}

float approxRollingAverage(float avg, float new_sample) {

  avg -= avg / Nsamples;
  avg += new_sample / Nsamples;

  return avg;
}

// Check if Wally is present
bool isWallyNearBy() {
  static float avgDistance = 0;
  int distance = sensor.readRangeSingleMillimeters();
  if (!sensor.timeoutOccurred()) {
    // filter noisy sensor readings by averaging the last Nsamples
    // ignore very short measurements
    if (distance < 100) {
      DPRINTLN("Ignoring short distance: " + String(distance));
      distance = 2 * DISTANCE_THSDL;
    }
    avgDistance = approxRollingAverage(avgDistance, distance);
  } else {
    DPRINTLN("Sensor timeout!");
  }

  DPRINT("Distance: " + String(distance) +
         " mm. Avg: " + String(avgDistance, 1));

  bool wallyNearby = (avgDistance <= DISTANCE_THSDL);
  if (wallyNearby) {
    wallyLastSeen = millis();
    DPRINTLN(" ->> Wally is here!");
    lockingStamp = 0;
    messageDuration = 0;
  } else {
    DPRINTLN(" ->> Wally is not here!");
  }

  return wallyNearby;
}

// Lock the screen function and return current time as a reference timestamp
unsigned long lockScreen() {
#ifndef DEBUG
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press('l');
  Keyboard.releaseAll();
#endif
  return millis();
}

void loop() {

  if (isWallyNearBy()) {
    // LED is always on when Wally is around
    pwmWrite(led_pin, 255); // 100% duty cycle, always on
    return;
  }

  // Wally is not around
  unsigned long elapsedTime = millis() - wallyLastSeen;

  if (elapsedTime < WALLY_TOUT) {
    // let's give Wally some time to get back
    // Led is blinking half of the time. Wally has time to get back
    pwmWrite(led_pin, 127); // 50% duty cycle, slow blink

#ifdef DEBUG
    // print the remaining time only at every second
    int remainingTime = (WALLY_TOUT - elapsedTime) / 1000;
    if (remainingTime != messageDuration) {

      DPRINTLN("Locking the screen in: " + String(remainingTime) + " seconds");
      messageDuration = remainingTime;
    }
#endif

  } else {
    // time for Wally to get back is gone, lock the screen
    if (!lockingStamp) {
      lockingStamp = lockScreen();
      DPRINTLN("Screen locked!");
      // Led is blinking briefly every second. Screen is locked
      pwmWrite(led_pin, 5); // 10% duty cycle, quick blink
    }

    // resending the lock screen command every minute
    unsigned long elapsedLockingTime = millis() - lockingStamp;
    if (elapsedLockingTime > 60000) {
      lockingStamp = lockScreen();
      DPRINTLN("Screen locked!");
    }
  }
}
