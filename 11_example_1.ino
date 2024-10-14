#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9    // LED active-low
#define PIN_TRIG  12   // sonar sensor TRIGGER
#define PIN_ECHO  13   // sonar sensor ECHO
#define PIN_SERVO 10   // servo motor

// Configurable parameters
#define SND_VEL 346.0     // sound velocity (m/sec) at 24°C
#define INTERVAL 25       // sampling interval (ms)
#define PULSE_DURATION 10 // pulse duration (usec)
#define TIMEOUT 25000     // max waiting time for echo (usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // scale to convert duration to distance (mm)

#define EMA_ALPHA 0.77 // Optimal alpha value for fast response and noise filtering
#define DIST_MIN 180.0 // 18 cm (unit: mm)
#define DIST_MAX 360.0 // 36 cm (unit: mm)

// Global variables
float dist_ema = DIST_MAX;
unsigned long last_sampling_time;

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);  // Sonar OFF

  myservo.attach(PIN_SERVO);
  myservo.write(0);  // Start at 0 degrees

  Serial.begin(57600);
}

void loop() {
  // Wait for the next sampling time
  if (millis() < (last_sampling_time + INTERVAL)) return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply EMA filter
  dist_ema = (dist_raw > 0) ? (EMA_ALPHA * dist_raw) + ((1 - EMA_ALPHA) * dist_ema) : dist_ema;

  // Apply range filter and control LED
  if (dist_ema >= DIST_MIN && dist_ema <= DIST_MAX) {
    digitalWrite(PIN_LED, LOW);  // LED ON
    float angle = map(dist_ema, DIST_MIN, DIST_MAX, 0, 180);  // Calculate angle
    myservo.write(angle);
  } else {
    digitalWrite(PIN_LED, HIGH); // LED OFF
    myservo.write(dist_ema < DIST_MIN ? 0 : 180); // 0° if below min, 180° if above max
  }

  // Output distance and angle to Serial Monitor
  Serial.print("Dist (mm): "); Serial.print(dist_ema);
  Serial.print(", Angle: "); Serial.println(myservo.read());

  // Update last sampling time
  last_sampling_time = millis();
}

// Function to measure distance with USS (in mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
