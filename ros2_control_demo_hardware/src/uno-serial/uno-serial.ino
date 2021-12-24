#include <Arduino.h>
#include <PID_v2.h>
// #include <L298N.h>

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// arduino Uno interrupt Pins: 2 and 3, Arduino Mega:  2, 3, 18, 19, 20, 21
// left
#define B1A 3   // Quadrature encoder A pin for right wheel
#define B1B 4
#define ENB 11  // black
#define IN3 12  // yellow
#define IN4 13  // green

// right
#define B2A 2   // black, Quadrature encoder A pin for right wheel
#define B2B 10  // blue
#define ENA 5 // brown
#define IN1 6 // red
#define IN2 7 // orange

// create Messenger object
// Messenger Messenger_Handler = Messenger();
void(*resetFunc)(void) = 0; //declare reset function at address 0

// L298N   lMotor(ENB, IN3, IN4), rMotor(ENA, IN1, IN2);
Encoder lEnc(B1A, B1B), rEnc(B2A, B2B);

// const double PPR = 4680.0; need to verify this
const double PPR = 2340.0;
const int WHEEL_RADIUS = 0.0325;
const float DEG_PER_TICK = 360.0 / PPR;
// float kp = 2.1, ki = 0.2 , kd = 0.5;          // modify for optimal performance
double kp = 0.35, ki = 0.475 , kd = 0.0;          // modify for optimal performance

double input[2] = {0}, output[2] = {0}, setpoint[2] = {0};
PID lPID(&input[0], &output[0], &setpoint[0], kp, ki, kd, PID::P_On::Measurement, PID::Direct);
PID rPID(&input[1], &output[1], &setpoint[1], kp, ki, kd, PID::P_On::Measurement, PID::Direct);

void rPwmOut(int out) {
  // drive motor CW
  digitalWrite(IN1, out > 0);
  digitalWrite(IN2, out < 0);
  analogWrite(ENA, abs(out));
}

void lPwmOut(int out) {
  double input = 0, output = 0, setpoint = 0;
  // drive motor CW
  digitalWrite(IN3, out < 0);
  digitalWrite(IN4, out > 0);
  analogWrite(ENB, abs(out));
}

void setup_motors() {
  // right
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // left
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

template <typename T>
void updateEncoders(T lData, T rData)
{
  Serial.print("e");
  Serial.print("\t");
  Serial.print(lData);
  Serial.print("\t");
  Serial.print(rData);

  // add filler to make it 9 bytes for pyseial in_waiting >= 9
  /*
    Serial.print("\t");
    Serial.print(0);
    Serial.print("\t");
    Serial.print(0);
  */
  Serial.print("\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise

  // deg.data_length=2;

  setup_motors();

  lEnc.write(0);
  rEnc.write(0);

  lPID.SetMode(PID::Automatic);
  // lPID.SetSampleTime(50);  // default 100
  lPID.SetOutputLimits(-255, 255);
  rPID.SetMode(PID::Automatic);
  rPID.SetOutputLimits(-255, 255);

  setpoint[0] = setpoint[1] = 0;
}

unsigned long lastTime = 0;
int32_t last_pos[2] = {0};
int32_t enc[2] = {0};
float lDeg, rDeg;

void loop() {
  unsigned long now = millis();
  unsigned int duration = (now - lastTime);

  // if not set to 100, rad/s won't reach to 3.0rad/s (i.e., 0.1m/s)
  if (duration >= 100 )
  {
    enc[0] = -1 * lEnc.read();
    enc[1] = rEnc.read();
    lastTime = now;

    if (enc[0] != last_pos[0] || enc[1] != last_pos[1]) {
      // they are in degrees not raidans yet.
      lDeg = (float)((enc[0] - last_pos[0]) * DEG_PER_TICK);
      rDeg = (float)((enc[1] - last_pos[1]) * DEG_PER_TICK);
      updateEncoders(lDeg, rDeg);

      // input[i] is in degree/s
      input[0] = 1000.0 * lDeg / duration;
      input[1] = 1000.0 * rDeg / duration;
      last_pos[0] = enc[0];
      last_pos[1] = enc[1];
    }

    lPID.Compute();
    lPwmOut((int)output[0]);
    rPID.Compute();
    rPwmOut((int)output[1]);
  }
}

void serialEvent()
{
  while (Serial.available()) {
    String command = Serial.readStringUntil(':'); // read commad
    Serial.read();  // read ":" out
    if (command == "s") {
      String inputStr = Serial.readStringUntil(',');
      Serial.read();  // read "," out
      setpoint[0] = inputStr.toInt();
      inputStr = Serial.readStringUntil('\r'); // read 2nd param
      Serial.read();  // read "/r" out
      setpoint[1] = inputStr.toInt();
    }
    else {
      Serial.readStringUntil('\r'); // start over
    }
  }
}
