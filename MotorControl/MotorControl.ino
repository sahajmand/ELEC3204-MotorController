#include <PID_v1.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// --- Pin Definitions ---
#define PWM_PIN            6
#define DIR_PIN1           4
#define DIR_PIN2           5
#define ENCODER_PIN_A      2
#define ENCODER_PIN_B      3
#define EMERGENCY_STOP_PIN 7
#define ESTOP_LED_PIN      8
#define MODE_SWITCH_PIN    9
#define POT_PIN           A0

// --- Motor Specs ---
const int   MAX_OUTPUT_RPM   = 100;
const int   ENCODER_PPR      = 12;
const int   GEAR_RATIO       = 99;
const int   PULSES_PER_REV   = ENCODER_PPR;
const int   MAX_MOTOR_RPM    = MAX_OUTPUT_RPM * GEAR_RATIO;

// --- PID Setup ---
double setRPM               = 0;
double absSetRPM            = 0;
double shaftSetRPM          = 0;
double currentMotorRPM      = 0;
double absCurrentMotorRPM   = 0;
double currentOutputRPM     = 0;
double pwmOutput            = 0;
double Kp = 0.075, Ki = 0.08, Kd = 0;
PID speedPID(&absCurrentMotorRPM, &pwmOutput, &absSetRPM, Kp, Ki, Kd, DIRECT);

// --- Encoder Counting ---
volatile long encoderTicks = 0;

// --- Motor State ---
volatile bool motorRunning   = false;
bool          motorForward   = true;
volatile int  lastPWMOut     = 0;

// --- Serial Input ---
String  inputString    = "";
bool    stringComplete = false;

// --- E-Stop Latch ---
bool emergencyLatched = false;

// --- Timer Setup ---
const unsigned long RPM_INTERVAL_MS = 50;  // Interval in ms
const unsigned int  TIMER1_TOP      = (unsigned int)(16.0e6 / 1024.0 * RPM_INTERVAL_MS / 1000.0);

// -----------------------------------------------
void setup() {
  // Pins
  pinMode(PWM_PIN,            OUTPUT);
  pinMode(DIR_PIN1,           OUTPUT);
  pinMode(DIR_PIN2,           OUTPUT);
  pinMode(ENCODER_PIN_A,      INPUT);
  pinMode(ENCODER_PIN_B,      INPUT);
  pinMode(EMERGENCY_STOP_PIN, INPUT);
  pinMode(MODE_SWITCH_PIN,    INPUT);
  pinMode(ESTOP_LED_PIN,      OUTPUT);
  pinMode(POT_PIN,            INPUT);


  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, RISING);

  // Serial
  Serial.begin(9600);
  inputString.reserve(50);

  // PID
  speedPID.SetOutputLimits(0, 255);
  speedPID.SetMode(AUTOMATIC);

  // Timer1 CTC interrupt every RPM_INTERVAL_MS
  cli();               // disable interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = TIMER1_TOP;
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);      // prescaler 1024
  TIMSK1 |= (1 << OCIE1A);                  // enable compare interrupt
  sei();               // enable interrupts
  
  digitalWrite(ESTOP_LED_PIN, LOW);

}

// -----------------------------------------------
void loop() {
  // Handle serial commands
  if (stringComplete) {
    handleCommand(inputString);
    inputString    = "";
    stringComplete = false;
  }

  // Emergency stop (latch)
  if (digitalRead(EMERGENCY_STOP_PIN) == HIGH && !emergencyLatched) {
    stopMotor();
    InformESTOP();
    setRPM          = 0;
    absSetRPM       = 0;
    emergencyLatched = true;
    Serial.println("EMERGENCY STOP TRIGGERED! System latched. Reset required.");
  }
}

// -----------------------------------------------
void encoderISR() {
  int b = digitalRead(ENCODER_PIN_B);
  if (b == LOW)  encoderTicks++;
  else           encoderTicks--;
}

// -----------------------------------------------
ISR(TIMER1_COMPA_vect) {
  // Determine control mode
  bool openLoopMode = (digitalRead(MODE_SWITCH_PIN) == LOW);  // LOW=open-loop, HIGH=closed-loop

  // Sample and reset tick count
  noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
  interrupts();

  // Compute RPMs
  currentMotorRPM    = (ticks / (double)PULSES_PER_REV) * (60000.0 / RPM_INTERVAL_MS);
  currentOutputRPM   = currentMotorRPM / GEAR_RATIO;
  shaftSetRPM        = setRPM / GEAR_RATIO;
  absCurrentMotorRPM = abs(currentMotorRPM);

  // Drive motor
  if (motorRunning && !emergencyLatched) {
    if (openLoopMode) {
      // Open-loop: map desired shaft RPM to PWM
      // float pct      = abs(shaftSetRPM) / MAX_OUTPUT_RPM;
      // lastPWMOut     = constrain((int)(pct * 255.0), 0, 255);
      int potValue = analogRead(POT_PIN);           // 0–1023
      lastPWMOut = map(potValue, 0, 1023, 0, 255);   // 0–255 PWM
    } else {
      // Closed-loop: PID
      speedPID.Compute();
      lastPWMOut = (int)pwmOutput;
    }
    analogWrite(PWM_PIN, lastPWMOut);
  } else {
    analogWrite(PWM_PIN, 0);
    lastPWMOut = 0;
  }

  // Publish exactly four ints: loop, direction, rpm, estop
  int loopFlag      = openLoopMode ? 0 : 1;         // 1=closed-loop, 0=open-loop
  int directionFlag = motorForward ? 0 : 1;         // 0=forward, 1=reverse
  int rpmInt        = (int)currentOutputRPM;       // shaft RPM
  int estopFlag     = emergencyLatched ? 1 : 0;     // 1=latched, 0=OK

  Serial.print(loopFlag);      Serial.print(',');
  Serial.print(directionFlag); Serial.print(',');
  Serial.print(rpmInt);        Serial.print(',');
  Serial.println(estopFlag);
}

// -----------------------------------------------
void startMotor() {
  if (emergencyLatched) return;
  motorRunning = true;
  updateDirection();
}

void stopMotor() {
  motorRunning = false;
  analogWrite(PWM_PIN, 0);
}

void InformESTOP() {
  digitalWrite(ESTOP_LED_PIN, HIGH);
}

void reverseMotor() {
  motorForward = !motorForward;
  updateDirection();
  setRPM        = -setRPM;
  absSetRPM     = abs(setRPM);
}

void updateDirection() {
  if (motorForward) {
    digitalWrite(DIR_PIN1, HIGH);
    digitalWrite(DIR_PIN2, LOW);
  } else {
    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, HIGH);
  }
}

// -----------------------------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// -----------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();
  if (emergencyLatched && !cmd.equalsIgnoreCase("STATUS")) {
    Serial.println("System latched. Reset required.");
    return;
  }

  if (cmd.equalsIgnoreCase("START")) {
    startMotor();
    Serial.println("Motor started");
  }
  else if (cmd.equalsIgnoreCase("STOP")) {
    stopMotor();
    Serial.println("Motor stopped");
  }
  else if (cmd.equalsIgnoreCase("REVERSE")) {
    reverseMotor();
    Serial.print("Direction reversed. New setpoint (shaft RPM): ");
    Serial.println(shaftSetRPM);
  }
  else if (cmd.startsWith("SPEED ")) {
    float ShaftRPM = cmd.substring(6).toFloat();         // may be negative
    float MotorRPM = ShaftRPM * GEAR_RATIO;
    MotorRPM       = constrain(MotorRPM, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    setRPM         = MotorRPM;
    absSetRPM      = abs(setRPM);
    motorForward   = (setRPM >= 0);
    updateDirection();
    Serial.print("Reference shaft RPM set to: ");
    Serial.println(ShaftRPM);
  }
  else if (cmd.equalsIgnoreCase("STATUS")) {
    Serial.print("Motor running: ");    Serial.println(motorRunning    ? "YES":"NO");
    Serial.print("Emergency latched: ");Serial.println(emergencyLatched? "YES":"NO");
    Serial.print("Mode: ");             Serial.println(digitalRead(MODE_SWITCH_PIN)==LOW? "OPEN-LOOP":"CLOSED-LOOP");
  }
  else {
    Serial.println("Unknown command");
  }
}
