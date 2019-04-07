/*

  MSE 2202 MSEBot Project
  Language: Arduino
  Authors: Group 16
  Date: current

  Rev 1 - Initial version
  Rev 2 - Update for MSEduino v. 2

*/


#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_leftMotor;
Servo servo_rightMotor;
Servo servo_slideMotor;
//Servo servo_rollerMotor;  //might just connect it to power

I2CEncoder encoder_leftMotor;
I2CEncoder encoder_rightMotor;

//pins
const int frontLeftPing = 4;   //trig
const int frontLeftData = 5;   //echo
const int frontRightPing = 10;
const int frontRightData = 11;
const int leftPing = 2;
const int leftData = 3;
const int rightPing = 12;
const int rightData = 13;
const int leftMotor = 8;
const int rightMotor = 9;
const int slideMotor = 6;
const int motorEnableSwitch = 7;
const int leftBumper = A1;  //will also control robot state
const int rightBumper = A0; //C to ground and NO to port
const int leftLightSensor = A2;
const int backLightSensor = A3;
const int ci_I2C_SDA = A4;         //I2C data = white
const int ci_I2C_SCL = A5;         //I2C clock = yellow

//constants
const int forwardSpeed = 1700; //set to max later (2100)
const int reverseSpeed = 1300; //set to max later (900)
const int forwardSlow = 1600;
const int reverseSlow = 1400;
const int brake = 1500;
const int rightAdjust = 10;
const int straightTolerance = 5;
const int wallTolerance = 1000;
const int wallAllowance = 1600;
const int beaconAllowance = 2500;
const int turnAllowance = 1600;
const int ninety = 575;
const int halfRobot = 1000;
const int displayTime = 500;
const int slideRetracted = 0;
const int slideExtended = 1800;
const int motorCalibrationCycles = 3;
const int motorCalibrationTime = 5000;
const int leftMotorOffsetAddress_L = 12;
const int leftMotorOffsetAddress_H = 13;
const int rightMotorOffsetAddress_L = 14;
const int rightMotorOffsetAddress_H = 15;

//variables
long leftMotorPosition;
long rightMotorPosition;
unsigned long tempTimer = 0;
unsigned long chargeTimer = 0;
unsigned long bumpTimer = 0;
unsigned long alignTimer = 0;
unsigned long searchTimer = 0;
unsigned long frontLeftEcho;
unsigned long frontRightEcho;
unsigned long leftEcho;
unsigned long rightEcho;
unsigned long tempLeftEcho;
unsigned long tempRightEcho;
unsigned long threeSecondTimer = 3000;
unsigned long tempDisplayTime;
unsigned long calibrationTime;
unsigned long leftMotorOffset;
unsigned long rightMotorOffset;
unsigned buttonHold;
unsigned robotState = 0;
unsigned phaseA = 1; //overall logic
unsigned phaseB = 1; //maneuvering walls or obstacles
unsigned phaseC = 1; //avoiding obstacles
unsigned dumbCount = 0;
unsigned leftMotorSpeed;
unsigned rightMotorSpeed;
unsigned calCount;
unsigned calCycle;
unsigned forwardTimer = 0;
unsigned backwardTimer = 0;
unsigned wiggleCount = 0;
bool wiggleFlag = false;
bool secondRound = false;
bool turnFlag = false;
bool leftBumpFlag = false;
bool rightBumpFlag = false;
bool alignFlag = false;
bool sideObstacle = false;
bool toggle = false;
bool worthFlag = false;
bool motorsEnabled = true;
bool bt_3sTimeUp = false;
bool bt_calInitialized = false;
byte b_lowByte;
byte b_highByte;


void setup() {
  Wire.begin();        //wire library required for I2CEncoder library
  Serial.begin(9600);

  //set up ultrasonic sensors
  pinMode(frontLeftPing, OUTPUT);
  pinMode(frontLeftData, INPUT);
  pinMode(frontRightPing, OUTPUT);
  pinMode(frontRightData, INPUT);
  pinMode(leftPing, OUTPUT);
  pinMode(leftData, INPUT);
  pinMode(rightPing, OUTPUT);
  pinMode(rightData, INPUT);

  //set up motors
  pinMode(rightMotor, OUTPUT);
  servo_rightMotor.attach(rightMotor);
  pinMode(leftMotor, OUTPUT);
  servo_leftMotor.attach(leftMotor);
  pinMode(slideMotor, OUTPUT);
  servo_slideMotor.attach(slideMotor);
  /*pinMode(rollerMotor, OUTPUT);           likely unnecessary
    servo_rollerMotor.attach(rollerMotor);*/

  //set up bumper and mode button
  pinMode(leftBumper, INPUT_PULLUP);
  pinMode(rightBumper, INPUT_PULLUP);
  pinMode(motorEnableSwitch, INPUT);

  //set up encoders
  encoder_leftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_leftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_rightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_rightMotor.setReversed(true);  // adjust for positive count when moving forward

  //read saved values from EEPROM
  b_lowByte = EEPROM.read(leftMotorOffsetAddress_L);
  b_highByte = EEPROM.read(leftMotorOffsetAddress_H);
  leftMotorOffset = word(b_highByte, b_lowByte);
  b_lowByte = EEPROM.read(rightMotorOffsetAddress_L);
  b_highByte = EEPROM.read(rightMotorOffsetAddress_H);
  rightMotorOffset = word(b_highByte, b_lowByte);
}


void loop() {
  if (((millis() - threeSecondTimer) >= 3000) && (robotState != 0)) //test
  {
    bt_3sTimeUp = true;
  }

  //button-based mode selection
  if ((digitalRead(leftBumper) == 0) && (toggle == false) && (robotState == 0))
  {
    tempTimer = millis();
    toggle = true;
    bt_3sTimeUp = false;
  }

  if ((digitalRead(leftBumper) == 1) && (toggle == true) && (robotState == 0))
  {
    buttonHold = millis() - tempTimer;
    toggle = false;
    threeSecondTimer = millis();

    if (buttonHold < 2000) //tap to run or hold to callibrate
    {
      robotState = 1;
    }

    else
    {
      robotState = 2;
    }
  }

  // check if drive motors should be powered
  motorsEnabled = digitalRead(motorEnableSwitch);

  // modes
  // 0 = default after power up/reset
  // 1 = tap left bumper to run
  // 2 = hold left bumper to callibrate
  switch (robotState)
  {
    case 0:    //default state
      leftEcho = ping(leftPing, leftData);
      rightEcho = ping(rightPing, rightData);
      servo_leftMotor.writeMicroseconds(brake);
      servo_rightMotor.writeMicroseconds(brake);
      servo_slideMotor.write(slideRetracted);
      encoder_leftMotor.zero();
      encoder_rightMotor.zero();
      break;

    case 1:    //run robot
      if (bt_3sTimeUp)
      {
        //set motor speeds
        leftMotorSpeed = constrain(forwardSpeed + leftMotorOffset, 1600, 2100);
        rightMotorSpeed = constrain(forwardSpeed + rightMotorOffset, 1600, 2100);
        Serial.println(phaseB);
        if ((millis() - chargeTimer >= 15000) && ((digitalRead(leftLightSensor) == 0) || (analogRead(leftLightSensor) < 500)
            || (digitalRead(backLightSensor) == 0) || (analogRead(backLightSensor) < 500)))     //start moving towards the beacon
        {
          phaseA = 3;
        }

        switch (phaseA)
        {
          case 1:
            randomDrive();
            break;

          case 2:
            maneuver();
            break;

          case 3:
            findBeacon();
            break;
        }

        if (motorsEnabled == true)
        {
          servo_leftMotor.writeMicroseconds(leftMotorSpeed);
          servo_rightMotor.writeMicroseconds(rightMotorSpeed);
        }

        if (motorsEnabled == false)
        {
          servo_leftMotor.writeMicroseconds(brake);
          servo_rightMotor.writeMicroseconds(brake);
        }
      }

      break;

    case 2:    //calibrate motors
      if (bt_3sTimeUp)
      {
        if (!bt_calInitialized)
        {
          bt_calInitialized = true;
          encoder_leftMotor.zero();
          encoder_rightMotor.zero();
          calibrationTime = millis();
          servo_leftMotor.writeMicroseconds(forwardSpeed);
          servo_rightMotor.writeMicroseconds(forwardSpeed);
        }

        else if ((millis() - calibrationTime) > motorCalibrationTime)
        {
          servo_leftMotor.writeMicroseconds(brake);
          servo_rightMotor.writeMicroseconds(brake);
          leftMotorPosition = encoder_leftMotor.getRawPosition();
          rightMotorPosition = encoder_rightMotor.getRawPosition();

          if (leftMotorPosition > rightMotorPosition)
          {
            // May have to update this if different calibration time is used
            rightMotorOffset = 0;
            leftMotorOffset = (leftMotorPosition - rightMotorPosition) / 4;
          }

          else
          {
            // May have to update this if different calibration time is used
            rightMotorOffset = (rightMotorPosition - leftMotorPosition) / 4;
            leftMotorOffset = 0;
          }

          EEPROM.write(rightMotorOffsetAddress_L, lowByte(rightMotorOffset));
          EEPROM.write(rightMotorOffsetAddress_H, highByte(rightMotorOffset));
          EEPROM.write(leftMotorOffsetAddress_L, lowByte(leftMotorOffset));
          EEPROM.write(leftMotorOffsetAddress_H, highByte(leftMotorOffset));
          robotState = 0;    //go back to Mode 0
        }
      }
      break;
  }

  if ((millis() - tempDisplayTime) > displayTime) //might be unnecessary
  {
    tempDisplayTime = millis();
  }
}


unsigned long ping(const int p, const int d)
{
  digitalWrite(p, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(p, LOW);
  return pulseIn(d, HIGH, 10000);
}


void randomDrive() {
  frontLeftEcho = ping(frontLeftPing, frontLeftData);
  frontRightEcho = ping(frontRightPing, frontRightData);

  if ((frontLeftEcho <= wallAllowance) || (frontRightEcho <= wallAllowance))  //wall or obstacle was reached
  {
    motorsEnabled = false;
    dumbCount++;
  }

  if (dumbCount >= 100) //maneuver the wall or obstacle
  {
    dumbCount = 0;
    phaseA = 2;
  }

  if ((alignFlag == true) && ((millis() - alignTimer) >= 2000))
  {
    dumbCount = 0;
    phaseA = 2;
  }

  else
  {
    leftMotorSpeed = forwardSpeed;
    rightMotorSpeed = forwardSpeed + rightAdjust;
    motorsEnabled = true;
  }
}


void maneuver() {
  switch (phaseB)
  {
    case 1: //check if there is space to turn, reverse if none
      leftEcho = ping(leftPing, leftData);

      if (leftEcho < turnAllowance)
      {
        leftMotorSpeed = reverseSpeed;
        rightMotorSpeed = reverseSpeed - rightAdjust;
        motorsEnabled = true;
        sideObstacle = true;
      }

      else
      {
        motorsEnabled = false;
        encoder_leftMotor.zero();
        encoder_rightMotor.zero();
        if (sideObstacle == true)
        {
          tempTimer = millis();
          phaseB = 3;
        }
        else
        {
          phaseB = 2;
        }
      }
      break;

    case 2:
      if ((encoder_rightMotor.getRawPosition() >= ninety) && (alignFlag == false))
      {
        motorsEnabled = false;
        alignTimer = millis();
        phaseB = 1;
        phaseA = 1;
      }

      if ((encoder_rightMotor.getRawPosition() >= 1200) && (alignFlag == true))
      {
        motorsEnabled = false;
        alignTimer = millis();
        phaseB = 1;
        phaseA = 1;
      }

      else
      {
        leftMotorSpeed = reverseSpeed;
        rightMotorSpeed = forwardSpeed + rightAdjust;
        motorsEnabled = true;
      }
      break;

    case 3: //reverse about half the robot's length
      if (((millis() - tempTimer) >= halfRobot) || (digitalRead(leftBumper) == 0) || (digitalRead(rightBumper) == 0))
      {
        motorsEnabled = false;
        leftEcho = ping(leftPing, leftData);
        rightEcho = ping(rightPing, rightData);
        encoder_leftMotor.zero();
        encoder_rightMotor.zero();
        sideObstacle = false;
        phaseB = 2;
      }

      else
      {
        leftMotorSpeed = reverseSpeed;
        rightMotorSpeed = reverseSpeed - rightAdjust;
        motorsEnabled = true;
      }
      break;
  }
}


void findBeacon() {
  switch (phaseC)
  {
    case 1:
      if ((digitalRead(backLightSensor) == 0) || (analogRead(backLightSensor) < 500))
      {
        motorsEnabled = false;
        tempTimer = millis();
        phaseC = 10;
      }

      else
      {
        leftMotorSpeed = reverseSpeed;
        rightMotorSpeed = forwardSpeed + rightAdjust;
        motorsEnabled = true;
      }
      break;

    case 10:
      if ((millis() - tempTimer) >= 75)
      {
        motorsEnabled = false;
        leftBumpFlag = false;
        rightBumpFlag = false;
        phaseC = 2;
      }

      else
      {
        leftMotorSpeed = forwardSpeed;
        rightMotorSpeed = reverseSpeed - rightAdjust;
        motorsEnabled = true;
      }
      break;

    case 2:
      if ((digitalRead(leftBumper) == 0) && (digitalRead(rightBumper) == 0)) //both bumpers were pressed
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      else
      {
        if ((digitalRead(leftBumper) == 0) && (leftBumpFlag == false)) //only left bumper was pressed
        {
          bumpTimer = millis();
          turnFlag = false;
          leftBumpFlag = true;
          leftMotorSpeed = reverseSpeed;
          rightMotorSpeed = reverseSpeed;
          motorsEnabled = true;
        }

        if ((digitalRead(rightBumper) == 0) && (rightBumpFlag == false))  //only right bumper was pressed
        {
          bumpTimer = millis();
          turnFlag = true;
          rightBumpFlag = true;
          leftMotorSpeed = reverseSpeed;
          rightMotorSpeed = reverseSpeed;
          motorsEnabled = true;
        }

        if (((millis() - bumpTimer) >= 1500) && ((leftBumpFlag == true) || (rightBumpFlag == true)))
        {
          forwardTimer = millis();
          backwardTimer = millis();
          phaseC = 4;
        }

        else
        {
          leftMotorSpeed = reverseSpeed;
          rightMotorSpeed = reverseSpeed - rightAdjust;
          motorsEnabled = true;
        }
      }
      break;

    case 3:
      servo_slideMotor.write(slideExtended);
      delay(3000);
      servo_slideMotor.write(slideRetracted);
      motorsEnabled = false;
      alignFlag = false;
      leftBumpFlag = false;
      rightBumpFlag = false;
      sideObstacle = false;

      chargeTimer = millis();
      phaseC = 1;
      phaseA = 1;
      break;

    case 4:
      if ((digitalRead(leftBumper) == 0) && (digitalRead(rightBumper) == 0)) //both bumpers were pressed
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      if (wiggleCount < 3)
      {
        if ((millis() - forwardTimer) >= 500)
        {
          motorsEnabled = false;
          forwardTimer = millis();
          backwardTimer = millis();
          phaseC = 5;
        }
        else
        {
          motorsEnabled = true;
          leftMotorSpeed = forwardSlow;
          rightMotorSpeed = forwardSpeed;
        }
      }

      else
      {
        wiggleCount = 0;
        leftBumpFlag = false;
        rightBumpFlag = false;
        alignFlag = true;
        alignTimer = millis();
        tempTimer = millis();
        phaseC = 6;
      }
      break;

    case 5:
      if ((digitalRead(leftBumper) == 0) && (digitalRead(rightBumper) == 0)) //both bumpers were pressed
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      if ((digitalRead(leftBumper) == 0) && (rightBumpFlag == true) && (digitalRead(rightBumper) == 0))
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      if ((digitalRead(rightBumper) == 0) && (leftBumpFlag == true) && (digitalRead(leftBumper) == 0))
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      if (wiggleCount >= 3)
      {
        wiggleCount = 0;
        leftBumpFlag = false;
        rightBumpFlag = false;
        alignFlag = true;
        alignTimer = millis();
        tempTimer = millis();
        phaseC = 6;
      }

      if ((millis() - backwardTimer) >= 750)
      {
        motorsEnabled = false;
        forwardTimer = millis();
        backwardTimer = millis();
        wiggleCount++;
        phaseC = 4;
      }

      if ((digitalRead(leftBumper) == 0) && (digitalRead(rightBumper) == 0)) //both bumpers were pressed
      {
        motorsEnabled = false;
        phaseC = 3;
      }

      else
      {
        motorsEnabled = true;
        leftMotorSpeed = reverseSpeed;
        rightMotorSpeed = reverseSpeed;
      }
      break;

    case 6:
      if ((digitalRead(leftBumper) == 0) && (digitalRead(rightBumper) == 0)) //both bumpers were pressed
      {
        motorsEnabled = false;

        phaseC = 3;
      }

      if ((millis() - tempTimer) >= 1500)
      {
        motorsEnabled = false;
        leftBumpFlag = false;
        rightBumpFlag = false;
        phaseC = 1;
        phaseA = 1;
      }

      else
      {
        if (turnFlag == false)
        {
          leftMotorSpeed = forwardSlow;
          rightMotorSpeed = forwardSpeed + 200;
          motorsEnabled = true;
        }
        else
        {
          leftMotorSpeed = forwardSpeed + 200;
          rightMotorSpeed = forwardSlow;
          motorsEnabled = true;
        }
      }
      break;
  }
}
