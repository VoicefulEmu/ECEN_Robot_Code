/********************************************************************
  ECEN 240/301 Lab Code
  Light-Following Robot

  The approach of this code is to use an architectured that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.


  Version History
  1.1.3       11 January 2023   Creation by Dr. Mazzeo and TAs from 2022 version

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include "Arduino.h"
#include <CapacitiveSensor.h>

//
// Compiler defines: the compiler replaces each name with its assignment
// (These make your code so much more readable.)
//

/***********************************************************/
// Hardware pin definitions
// Replace the pin numbers with those you connect to your robot

// Button pins. These will be replaced with the photodiode variables in lab 5
#define BUTTON_1 A2  // Far left Button - Servo Up
#define BUTTON_2 A3  // Left middle button - Left Motor
#define BUTTON_3 A4  // Middle Button - Collision
#define BUTTON_4 A5  // Right middle button - Right Motor
#define BUTTON_5 A6  // Far right button - Servo Down

// LED pins (note that digital pins do not need "D" in front of them)
#define LED_1 6  // Far Left LED - Servo Up
// #define LED_2 5  // Left Middle LED  - Left Motor
#define LED_3 4  // Middle LED - Collision
// #define LED_4 3  // Right Middle LED - Right Motor
#define LED_5 2  // Far Right LED - Servo Down
#define TEST_LED 9 //used for testing 


// Motor enable pins - Lab 3
#define H_BRIDGE_ENA 5
#define H_BRIDGE_ENB 3
// These will replace LEDs 2 and 4

// Photodiode pins - Lab 5
// These will replace buttons 1, 2, 4, 5

// Capacitive sensor pins - Lab 4

#define CAP_SENSOR_SEND     11
#define CAP_SENSOR_RECEIVE  7

// Ultrasonic sensor pin - Lab 6
// This will replace button 3 and LED 3 will no longer be needed

// Servo pin - Lab 6
// This will replace LEDs 1 and 5

/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Voltage at which a button is considered to be pressed
#define BUTTON_THRESHOLD 2.5

// Voltage at which a photodiode voltage is considered to be present - Lab 5


// Number of samples that the capacitor sensor will use in a measurement - Lab 4
#define CAP_SENSOR_SAMPLES 40
#define CAP_SENSOR_TAU_THRESHOLD 100

// Parameters for servo control as well as instantiation - Lab 6


// Parameters for ultrasonic sensor and instantiation - Lab 6


// Parameter to define when the ultrasonic sensor detects a collision - Lab 6



/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO 0
#define DETECTION_YES 1

// Motor speed definitions - Lab 4
#define SPEED_STOP      0
#define SPEED_LOW       (int) (255 * 0.33)
#define SPEED_MED       (int) (255 * 0.66)
#define SPEED_HIGH      (int) (255 * 1)


// Collision definitions
#define COLLISION_ON 0
#define COLLISION_OFF 1

// Driving direction definitions
#define DRIVE_STOP 0
#define DRIVE_LEFT 1
#define DRIVE_RIGHT 2
#define DRIVE_STRAIGHT 3

// Servo movement definitions
#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_UP 1
#define SERVO_MOVE_DOWN 2


/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision = DETECTION_NO;

// Photodiode inputs (using Definitions) - The button represent the photodiodes for lab 2
int SensedLightRight = DETECTION_NO;
int SensedLightLeft = DETECTION_NO;
int SensedLightUp = DETECTION_NO;
int SensedLightDown = DETECTION_NO;

// Capacitive sensor input (using Definitions) - Lab 4
int SensedCapacitiveTouch = DETECTION_NO;
// long tau = 0;


/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STOP;
// Add speed action in Lab 4
int ActionRobotSpeed = SPEED_STOP;


// Servo Action (using Definitions)
int ActionServoMove = SERVO_MOVE_STOP;

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {
  //Set up serial connection at 9600 Baud
  Serial.begin(9600);

  //Set up output pins
  pinMode(LED_1, OUTPUT);
  pinMode(H_BRIDGE_ENA, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(H_BRIDGE_ENB, OUTPUT);
  pinMode(LED_5, OUTPUT);

  //Set up input pins
  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_3, INPUT);
  pinMode(BUTTON_4, INPUT);
  pinMode(BUTTON_5, INPUT);

  //Set up servo - Lab 6
}

/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/
void loop() {
  // This DebugStateOutput flag can be used to easily turn on the
  // serial debugging to know what the robot is perceiving and what
  // actions the robot wants to take.
  int DebugStateOutput = true;  // Change false to true to debug

  RobotPerception();  // PERCEPTION
  if (DebugStateOutput) {
    // Serial.print("Perception:");
    // Serial.print(SensedLightUp);
    // Serial.print(SensedLightLeft);
    // Serial.print(SensedCollision);
    // Serial.print(SensedLightRight);
    // Serial.print(SensedLightDown);
       Serial.print(SensedCapacitiveTouch); //- Lab 4
  }

  RobotPlanning();  // PLANNING
  if (DebugStateOutput) {
    Serial.print(" Action:");
    // Serial.print(ActionCollision);
    Serial.print(ActionRobotDrive);
       Serial.print(ActionRobotSpeed);// - Lab 4
    // Serial.println(ActionServoMove);
  }
  RobotAction();  // ACTION
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages



  // Photodiode Sensing
  // Serial.println(getPinVoltage(BUTTON_2)); //uncomment for debugging


  if (isButtonPushed(BUTTON_2)) {
    SensedLightLeft = DETECTION_YES;
  } else {
    SensedLightLeft = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (isButtonPushed(BUTTON_4)) {
    SensedLightRight = DETECTION_YES;
  } else {
    SensedLightRight = DETECTION_NO;
  }
  if (isButtonPushed(BUTTON_1)){
    SensedLightUp = DETECTION_YES;
  } else{
    SensedLightUp = DETECTION_NO;
  }
  if (isButtonPushed(BUTTON_5)){
    SensedLightDown = DETECTION_YES;
  } else{
    SensedLightDown = DETECTION_NO;
  }


  /* Add code to detect if light is up or down. Lab 2 milestone 3*/



  // Capacitive Sensor
  static CapacitiveSensor sensor 
    = CapacitiveSensor(CAP_SENSOR_SEND, CAP_SENSOR_RECEIVE);
  long tau 
    =  sensor.capacitiveSensor(CAP_SENSOR_SAMPLES); 

  SensedCapacitiveTouch = isCapacitiveSensorTouched(tau);
  // Serial.println(tau);


  // Collision Sensor
  if (isCollision()) {
    SensedCollision = DETECTION_YES;
  } else {
    SensedCollision = DETECTION_NO;
  }
}


////////////////////////////////////////////////////////////////////
// Function to read pin voltage
////////////////////////////////////////////////////////////////////
float getPinVoltage(int pin) {
  //This function can be used for many different tasks in the labs
  //Study this line of code to understand what is going on!!
  //What does analogRead(pin) do?
  //Why is (float) needed?
  //Why divide by 1024?
  //Why multiply by 5?
  return 5 * (float)analogRead(pin) / 1024;
}

////////////////////////////////////////////////////////////////////
// Function to determine if a button is pushed or not
////////////////////////////////////////////////////////////////////
bool isButtonPushed(int button_pin) {
  //This function can be used to determine if a said button is pushed.
  //Remember that when the voltage is 0, it's only close to zero.
  //Hint: Call the getPinVoltage function and if that value is greater
  // than the BUTTON_THRESHOLD variable toward the top of the file, return true.
  if (getPinVoltage(button_pin) > BUTTON_THRESHOLD) {
    return true;
  } else {
    return false;
  }
}



////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool isCollision() {
  //This is where you add code that tests if the collision button
  // was pushed (BUTTON_3)
  //In lab 6 you will add a sonar sensor to detect collision and
  // the code for the sonar sensor will go in this function.
  // Until then we will use a button to model the sensor.
  if (isButtonPushed(BUTTON_3)) {
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if the capacitive sensor is being touched
////////////////////////////////////////////////////////////////////
bool isCapacitiveSensorTouched(long tau) {
  //In lab 4 you will add a capacitive sensor, and
  // you will need to modify this function accordingly.
  if (tau > CAP_SENSOR_TAU_THRESHOLD) {
    SensedCapacitiveTouch = DETECTION_YES;
    return SensedCapacitiveTouch;
    // digitalWrite(TEST_LED, HIGH);
  } else {
    SensedCapacitiveTouch = DETECTION_NO;
    return SensedCapacitiveTouch;
    // digitalWrite(led_pin, LOW);
  }
}


/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void RobotPlanning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsmCollisionDetection();  // Milestone 1
  fsmMoveServoUpAndDown();  // Milestone 3
  fsmCapacitiveSensorSpeedControl();// Add Speed Control State Machine in lab 4
}

////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsmCollisionDetection() {
  static int collisionDetectionState = 0;
  //Serial.println(collisionDetectionState); //uncomment for debugging

  switch (collisionDetectionState) {
    case 0:  //collision detected
      //There is an obstacle, stop the robot
      ActionCollision = COLLISION_ON;  // Sets the action to turn on the collision LED
      ActionRobotDrive = DRIVE_STOP;


      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        collisionDetectionState = 1;  //if no collision, go to no collision state
      }
      break;

    case 1:  //no collision
      //There is no obstacle, drive the robot
      ActionCollision = COLLISION_OFF;  // Sets action to turn off the collision LED

      fsmSteerRobot();  // Milestone 2

      //State transition logic
      if (SensedCollision == DETECTION_YES) {
        collisionDetectionState = 0;  //if collision, go to collision state
      }
      break;

    default:  // error handling
      {
        collisionDetectionState = 0;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is to the right or left,
// and steering the robot accordingly.
////////////////////////////////////////////////////////////////////
void fsmSteerRobot() {
  static int steerRobotState = 0;
  //Serial.println(steerRobotState); //uncomment for debugging

  switch (steerRobotState) {
    case 0:  //light is not detected
      ActionRobotDrive = DRIVE_STOP;

      //State transition logic
      if (SensedLightLeft == DETECTION_YES) {
        steerRobotState = 1;  //if light on left of robot, go to left state
      } else if (SensedLightRight == DETECTION_YES) {
        steerRobotState = 2;  //if light on right of robot, go to right state
      }
      break;

    case 1:  //light is to the left of robot
      //The light is on the left, turn left
      ActionRobotDrive = DRIVE_LEFT;

      //State transition logic
      if (SensedLightRight == DETECTION_YES) {
        steerRobotState = 3;  //if light is on right, then go straight
      } else if (SensedLightLeft == DETECTION_NO) {
        steerRobotState = 0;  //if light is not on left, go back to stop state
      }

      break;

    case 2:  //light is to the right of robot
      //The light is on the right, turn right
      ActionRobotDrive = DRIVE_RIGHT;

      //State transition logic
      // *Add code to transition to the "light on right and left" state
      if (SensedLightLeft == DETECTION_YES) {
        steerRobotState = 3;
      } else if (SensedLightRight == DETECTION_NO) {
        steerRobotState = 0;  //if light is not on left, go back to stop state
      }

      break;


    case 3:  // light is on both right and left

      ActionRobotDrive = DRIVE_STRAIGHT;

      //State transition logic
      // *Add code to transition to the "light on right" state, light on left state, no  light state
      if (SensedLightLeft == DETECTION_NO) {
        steerRobotState = 2;
      } else if (SensedLightRight == DETECTION_NO) {
        steerRobotState = 1;
      } else if (SensedLightLeft && SensedLightRight == DETECTION_NO) {
        steerRobotState = 1;
      }

      break;

    default:  // error handling
      {
        steerRobotState = 0;
      }
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is above or below center,
// and moving the servo accordingly.
////////////////////////////////////////////////////////////////////
void fsmMoveServoUpAndDown() {
  static int moveServoState = 0;
  Serial.println(moveServoState);  //uncomment for debugging

  // Milestone 3
  //Create a state machine modeled after the ones in milestones 1 and 2
  // to plan the servo action based off of the perception of the robot
  //Remember no light or light in front = servo doesn't move
  //Light above = servo moves up
  //Light below = servo moves down
  switch (moveServoState) {
    case 0:  //light in front or not detected
      ActionServoMove = SERVO_MOVE_STOP;

      //State transition logic
      if (SensedLightDown == DETECTION_YES && SensedLightUp == DETECTION_NO) {
        moveServoState = 1;  //if light only below robot, go to point down state
      } else if (SensedLightDown == DETECTION_NO && SensedLightUp == DETECTION_YES) {
        moveServoState = 2;  //if light only above robot, go to point up state
      }
      break;

    case 1:  //point down state (light only below robot)
      ActionServoMove = SERVO_MOVE_DOWN;

      //State transition logic
      if ((SensedLightDown == DETECTION_YES && SensedLightUp == DETECTION_YES) || (SensedLightDown == DETECTION_NO && SensedLightUp == DETECTION_NO)) {
        moveServoState = 0;  //if light straight or gone, go to stopped state
      }
      break;

    case 2:  //point up state (light only above robot)
      ActionServoMove = SERVO_MOVE_UP;

      //State transition logic
      if ((SensedLightDown == DETECTION_YES && SensedLightUp == DETECTION_YES) || (SensedLightDown == DETECTION_NO && SensedLightUp == DETECTION_NO)) {
        moveServoState = 0;  //if light straight or gone, go to stopped state
      }
      break;
  }
}

  ////////////////////////////////////////////////////////////////////
  // State machine for detecting when the capacitive sensor is
  // touched, and changing the robot's speed.
  ////////////////////////////////////////////////////////////////////
  void fsmCapacitiveSensorSpeedControl() {
    static int SpeedControlState;
    /*Implement in lab 4*/
    switch (SpeedControlState)  {
    case 0:  //wait for button press
      //action logic
      //State transition logic
      if (SensedCapacitiveTouch) {
        SpeedControlState = 1;  //wait for release state
      }
      break;
      case 1: //wait for release
      //action logic
      //state transition logic
      if (! SensedCapacitiveTouch) {
        SpeedControlState = 2; //move to toggle speed
      }
      break;
      case 2: //toggle speed
      //action logic
      fsmChangeSpeed();
      //state transition logic
      SpeedControlState = 0;
  }
  }

  ////////////////////////////////////////////////////////////////////
  // State machine for cycling through the robot's speeds.
  ////////////////////////////////////////////////////////////////////
  void fsmChangeSpeed() {
    /*Implement in lab 4*/
    static int ChangeSpeedState;
    
    switch (ChangeSpeedState){
      case 0: //speed off
      //action logic
      ActionRobotSpeed = SPEED_STOP;
      //state transition logic
      ChangeSpeedState = 1;
      break;
      case 1:
      //action logic
      ActionRobotSpeed = SPEED_LOW;
      //state transition logic
      ChangeSpeedState = 2;
      break;
      case 2:
      //action logic
      ActionRobotSpeed = SPEED_MED;
      //state transition logic
      ChangeSpeedState = 3;
      break;
      case 3:
      //action logic
      ActionRobotSpeed = SPEED_HIGH;
      //state transition logic
      ChangeSpeedState = 0;
      break;
    }

  }


  /**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
  void RobotAction() {
    // Here the results of planning are implented so the robot does something

    // This turns the collision LED on and off
    switch (ActionCollision) {
      case COLLISION_OFF:
        doTurnLedOff(LED_3);  //Collision LED off - DON'T FORGET TO ADD CODE TO doTurnLedOff()
                              // AND doTurnLedOn() OR ELSE YOUR LEDS WON'T WORK!!!
        break;
      case COLLISION_ON:
        doTurnLedOn(LED_3);
        break;
    }

    // This drives the main motors on the robot
    switch (ActionRobotDrive) {
      case DRIVE_STOP:
        analogWrite(H_BRIDGE_ENA,0);
        analogWrite(H_BRIDGE_ENB,0);
        /* DON'T FORGET TO USE YOUR LED VARIABLES AND NOT YOUR BUTTON VARIABLES FOR THIS!!! */
        break;
      case DRIVE_LEFT:
        analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
        analogWrite(H_BRIDGE_ENB, 0);
        break;
      case DRIVE_RIGHT:
        analogWrite(H_BRIDGE_ENA, 0);
        analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
        break;
      case DRIVE_STRAIGHT:
        analogWrite(H_BRIDGE_ENA,ActionRobotSpeed);
        analogWrite(H_BRIDGE_ENB,ActionRobotSpeed);
        break;
    }

    // This calls a function to move the servo
    MoveServo();
  }


  ////////////////////////////////////////////////////////////////////
  // Function that causes the servo to move up or down.
  ////////////////////////////////////////////////////////////////////
  void MoveServo() {
    // Note that there needs to be some logic in the action of moving
    // the servo so that it does not exceed its range
    /* Add CurrentServoAngle in lab 6 */
    switch (ActionServoMove) {
      case SERVO_MOVE_STOP:
        doTurnLedOff(LED_1);
        doTurnLedOff(LED_5);
        break;
      case SERVO_MOVE_UP:
        doTurnLedOn(LED_1);
        break;
      case SERVO_MOVE_DOWN:
        doTurnLedOn(LED_5);
        break;
    }
  }



  /**********************************************************************************************************
  AUXILIARY functions that may be useful in performing diagnostics
 ********************************************************************/
  // Function to turn LED on
  void doTurnLedOn(int led_pin) {
    digitalWrite(led_pin, HIGH);
  }

  // Function to turn LED off
  void doTurnLedOff(int led_pin) {
    digitalWrite(led_pin, LOW);
  }
