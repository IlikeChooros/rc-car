#include <SoftwareSerial.h>
#include <Servo.h>


// ----------------------------------------------------------------
// ------------------------ DEFINES -------------------------------

// For debugging purposes, will send the data back to the bluetooth
constexpr bool DEBUG = false;

#define IF_ON_DEBUG if constexpr (DEBUG)
#define START_MSG "*"
#define END_MSG ";"
#define DATA_FORMAT "%d %d"
#define MSG_FORMAT START_MSG""DATA_FORMAT""END_MSG


// ----------------------------------------------------------------
// ------------------------ STRUCTS -------------------------------

// For motor controlling
struct MotorInfo
{
  uint8_t pwm;
  uint8_t prev_pwm;
  bool forward;
  bool prev_forward;
  uint8_t pin_pwm;
  uint8_t pin_in1;
  uint8_t pin_in2;
};

// For servo controlling
struct ServoInfo
{
  uint8_t angle; // 0 - 180
  uint8_t prev_angle;
  Servo servo;
};

// Must be the same as the server
typedef struct 
{
  int pwm;
  int rotation;
} MotorData;

// ----------------------------------------------------------------
// ------------------------ GLOBAL VARIABLES ----------------------

//Create software serial object to communicate with HC-05
SoftwareSerial btSerial(3, 2); //HC-05 Tx & Rx is connected to Arduino #3 & #2

// For motor controlling
static MotorInfo motor1     = {0, 0, false, true, 6, 7, 8};
static ServoInfo servo      = {90, 90, Servo()};
static MotorData motorData  = {0, 90};
static uint64_t  servoTime  = 0;
static uint64_t  servoDelay = 1000;

// For the loop
constexpr int    MAX_LEN     = 16;
static uint8_t   loopTimer   = 0;
static int       buffer_len  = 0;
static bool      valid_start = false;
static bool      valid_end   = false;
static char      buffer[MAX_LEN];


// ----------------------------------------------------------------
// ------------------------ FUNCTIONS -----------------------------

// Set the output modes on the pins
void setupMotorPinModes(MotorInfo& motor) 
{
  pinMode(motor.pin_pwm, OUTPUT);
  pinMode(motor.pin_in1, OUTPUT);
  pinMode(motor.pin_in2, OUTPUT);
}

// Set the motor rotation, (forward is in regard of the black part of the engine)
// If it's directed backwards, it will go forward
void setMotorRotation(bool forward, MotorInfo& motor)
{
  if (motor.forward == forward)
    return;

  motor.prev_forward = motor.forward;
  motor.forward = forward;
  uint8_t stateIn1 = LOW, stateIn2 = HIGH;

  if (!forward)
  {
    stateIn1 = !stateIn1;
    stateIn2 = !stateIn2;
  }
  
  digitalWrite(motor.pin_in1, stateIn1);
  digitalWrite(motor.pin_in2, stateIn2);
}

// analog wirte the pwm values
void updateMotorState()
{
  if (motor1.pwm != motor1.prev_pwm)
  {
    analogWrite(motor1.pin_pwm, motor1.pwm);
    motor1.prev_pwm = motor1.pwm;
  }
}

// Set the angle of the servo
void updateServoState()
{
  // Wait for the servo to reach the desired angle and change it if needed
  if ((millis() - servoTime < servoDelay))
    return;

  if (servo.angle == servo.prev_angle)
    return;

  IF_ON_DEBUG
  {
    String msg = "read-angle: " + String(servo.servo.read()) + "\n";
    btSerial.write(msg.c_str());
  }

  IF_ON_DEBUG
  {
    String msg = "w-angle: " + String(servo.angle) + "\n";
    btSerial.write(msg.c_str());
  }

  servo.servo.write(servo.angle);

  servoTime  = millis();
  servoDelay = max(5 * abs(servo.angle - servo.prev_angle), 30);
  servo.prev_angle = servo.angle;
}

// Update all states
void updateStates()
{
  updateMotorState();
  updateServoState();
}

// Setup all needed parts for the motor to work
void setupMotor()
{
  setupMotorPinModes(motor1);
  setMotorRotation(true, motor1);
}

// Setup all needed parts for the servo to work
void setupServo()
{
  servo.angle = 90;
  servo.prev_angle = 90;
  servo.servo.attach(A0);
  servo.servo.write(servo.angle);
}

// Get the memory left
uint16_t memoryLeft()
{
  extern int __heap_start, *__brkval;
  int v;
  return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup()
{
  //Begin serial communication with Arduino and HC-05
  btSerial.begin(9600);

  // Setup motor
  setupMotor();

  // Setup servo
  setupServo();
}

// Write the motor data to the motor
void writeMotorData()
{
// Prepare the pwm value
  if (motorData.pwm < 0)
  {
    // rotate backwards
    setMotorRotation(false, motor1);
    motorData.pwm = -motorData.pwm;
  }
  else
  {
    setMotorRotation(true, motor1);
  }
  // Set the value of the pwm
  motor1.pwm = motorData.pwm;

  // Set the servo angle
  servo.angle = motorData.rotation;
}

void loop()
{
  constexpr uint8_t LOOP_DELAY = 2;

  // Update the states
  updateStates();

  // Check the serial every 2 ms (500 Hz)
  if (millis() - loopTimer < LOOP_DELAY)
    return;

  loopTimer = millis();

  // Read the data from the bluetooth if available
  if(btSerial.available()) 
  {
    buffer_len = 0;
    valid_start = valid_end = false;

    // Wait for full message to be received
    delay(10);

    while(btSerial.available())
    {
      int data = btSerial.read();

      if (data == START_MSG[0])
      {
        valid_start = true;
        continue;
      }

      if (!valid_start)
        continue;

      if (data == END_MSG[0])
      {
        valid_end = true;
        break;
      }

      if (++buffer_len < (MAX_LEN - 1))
        buffer[buffer_len - 1] = data;
    }

    IF_ON_DEBUG
    {
      btSerial.write(buffer, buffer_len);
    }
    
    if (!(valid_end && valid_start))
      return;

    // Add the end of the string
    buffer[buffer_len] = '\0';

    // For debugging purposes only
    IF_ON_DEBUG
    {
      // Read the message and set the motor pwm and rotation
      int scanned = sscanf(buffer, DATA_FORMAT, &motorData.pwm, &motorData.rotation);

      // If the message is valid, set the motor pwm and rotation
      if (scanned == 2)
      {
        btSerial.write("\n");
        btSerial.println("pwm: " + String(motorData.pwm) + " r: " + String(motorData.rotation));
        writeMotorData();
      }
    }

    // For normal operation
    else
    {
      // Read the message and set the motor pwm and rotation
      int scanned = sscanf(buffer, DATA_FORMAT, &motorData.pwm, &motorData.rotation);

      // If the message is valid, set the motor pwm and rotation
      if (scanned == 2)
      {
        writeMotorData();
      }
    }
  }
}