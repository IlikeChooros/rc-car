#include <SoftwareSerial.h>
#include <Servo.h>

constexpr bool DEBUG = false;

#define IF_ON_DEBUG if constexpr (DEBUG)

//Create software serial object to communicate with HC-05
SoftwareSerial btSerial(3, 2); //HC-05 Tx & Rx is connected to Arduino #3 & #2

// For motor controlling
struct MotorInfo{
  uint8_t pwm;
  uint8_t prev_pwm;
  bool forward;
  bool prev_forward;
  uint8_t pin_pwm;
  uint8_t pin_in1;
  uint8_t pin_in2;
};

// For servo controlling
struct ServoInfo{
  uint8_t angle; // 0 - 180
  uint8_t prev_angle;
  Servo servo;
};

// Must be the same as the server
typedef struct {
  int16_t pwm;
  uint8_t rotation;
} MotorData;

static MotorInfo motor1 = {0, 0, false, true, 6, 7, 8};
static ServoInfo servo  = {90, 90, Servo()};
static MotorData motorData = {0, 90};

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
  if (servo.angle != servo.prev_angle)
  {
    servo.servo.write(servo.angle);
    servo.prev_angle = servo.angle;
  }
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
  motor1.pwm = min(255, motorData.pwm);

  // Set the servo angle
  servo.angle = min(180, motorData.rotation);
}

void loop()
{
  // Read the data from the bluetooth if available
  if(btSerial.available()) 
  {
    // Wait for full message to be received
    delay(20);


    // For debugging purposes only
    IF_ON_DEBUG
    {
      String s;
      while(btSerial.available())
      {
        int data = btSerial.read();
        if (isalnum(data) || data == '-' || data == ' ')
          s += (char) data;
      }

      // Read the message and set the motor pwm and rotation
      btSerial.write(s.c_str());

      int pwm, rotation;
      int scanned = sscanf(s.c_str(), "%d %d", &pwm, &rotation);

      // If the message is valid, set the motor pwm and rotation
      if (scanned == 2)
      {
        motorData.pwm = pwm;
        motorData.rotation = rotation;
        btSerial.write("OK\n");
        writeMotorData();
      }
    }

    // For normal operation
    else
    {
      // Read the message and set the motor pwm and rotation
      // int scanned = btSerial.readBytes((char*)&motorData, sizeof(motorData));

      String s;
      while(btSerial.available())
      {
        int data = btSerial.read();
        if (isalnum(data) || data == '-' || data == ' ')
          s += (char) data;
      }

      int pwm, rotation;
      int scanned = sscanf(s.c_str(), "%d %d", &pwm, &rotation);

      // If the message is valid, set the motor pwm and rotation
      if (scanned == 2)
      {
        motorData.pwm = pwm;
        motorData.rotation = rotation;
        writeMotorData();
      }
    }
  }
  updateStates();

  delay(5);
}