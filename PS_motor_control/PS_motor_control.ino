// Load libraries used
#include "SimpleRSLK.h"
#include <Servo.h>
#include "PS2X_lib.h"

// Define pin numbers for the button on the PlayStation controller
#define PS2_DAT 33  //P1.7 <-> brown wire
#define PS2_CMD 26  //P1.6 <-> orange wire
#define PS2_SEL 5   //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK 2   //P6.7 <-> blue wire

// Create an instance of the playstation controller object
PS2X ps2x;

// Define remote mode either playstation controller or IR remote controller
enum RemoteMode {
  PLAYSTATION,
  IR_REMOTE,
};

// Declare and initialize the current state variable
RemoteMode CurrentRemoteMode = PLAYSTATION;

// Tuning Parameters
const uint16_t lowSpeed = 15;
const uint16_t fastSpeed = 30;

Servo myservo;          // create servo object to control a servo
int gripper_pos = 139;  // Gripper starts open

void setup() {
  Serial.begin(57600);
  Serial.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();
  myservo.attach(SRV_0);  // This is pin 38, from RSLK_Pins.h
    // using the playstation controller
    Serial.println("Using playstation controller, make sure it is paired first ");

    // Initialize PlayStation controller
    delayMicroseconds(500 * 1000);  //added delay to give wireless ps2 module some time to startup, before configuring it
    // declare variables for playstation control
    bool pressures = false;
    bool rumble = false;
    int error = 1;

    while (error) {
      error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

      if (error == 0)
        Serial.println("Found Controller, configured successful ");

      else if (error == 1)
        Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

      else if (error == 2)
        Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

      else if (error == 3)
        Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
      delayMicroseconds(1000 * 1000);
    }

  
}

void loop() {
  // Read input from PlayStation controller
  ps2x.read_gamepad();

  // Operate the robot in remote control mode
  if (CurrentRemoteMode == 0) {
    Serial.println("Running remote control with the Playstation Controller");
    RemoteControlPlaystation();

  } else if (CurrentRemoteMode == 1) {
    // put code here to run using the IR controller if neccessary
  }
}

/* Moves robot forward: both motors forward same speed */
void forward() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
/* Moves robot backwards: both motors forward same speed */
void backwards() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
/* Moves robot spin right: both motors forward same speed */
void right_spin() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
/* Moves robot spin left: both motors forward same speed */
void left_spin() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
/* Moves robot turn right: both motors forward same speed */
void right_turn() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, fastSpeed);
  setMotorSpeed(RIGHT_MOTOR, lowSpeed);
}
/* Moves robot turn left: both motors forward same speed */
void left_turn() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  setMotorSpeed(LEFT_MOTOR, lowSpeed);
}
/* Opens the gripper */
void open() {
  for (int pos = gripper_pos; pos < 140; pos += 1)  // goes from 0 degrees to 180 degrees
  {                                                 // in steps of 1 degree
    myservo.write(pos);                             // tell servo to go to position in variable 'pos'
    delay(15);                                      // waits 15 ms for the servo to reach the position
  }
  gripper_pos = 41;
}
/* Closes the gripper */
void close() {
  for (int pos = gripper_pos; pos >= 41; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                               // tell servo to go to position in variable 'pos'
    delay(15);                                        // waits 15 ms for the servo to reach the position
  }
  gripper_pos = 139;
}
/* Stops robot forward: both motors disabled */
void stop() {
  disableMotor(BOTH_MOTORS);
}

/* RemoteControlPlaystation() function
  This function uses a playstation controller and the PLSK libraray with
  an RLSK robot using to implement remote controller. 
  
  A few actions are programed for an example. 

  Button control map:
  PAD UP button moves both motors forward
  CROSS button stops motors
  */
void RemoteControlPlaystation() {
  // put your code here to run in remote control mode

  // Example of receive and decode remote control command
  // the forward() and stop() functions should be independent of
  // the control methods
  if (ps2x.Button(PSB_PAD_UP)) {
    Serial.println("PAD UP button pushed ");
    forward();
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.println("PAD DOWN button pushed ");
    backwards();
  } else if (ps2x.Button(PSB_CROSS)) {
    Serial.println("CROSS button pushed");
    stop();
  } else if (ps2x.Button(PSB_SQUARE)) {
    Serial.println("SQUARE button pushed");
    open();
  } else if (ps2x.Button(PSB_TRIANGLE)) {
    Serial.println("TRIANGLE button pushed");
    close();
  } else if (ps2x.Button(PSB_L2)) {
    Serial.println("L2 button pushed");
    left_turn();
  } else if (ps2x.Button(PSB_R2)) {
    Serial.println("R2 button pushed");
    right_turn();
  } else if (ps2x.Button(PSB_L1)) {
    Serial.println("L1 button pushed");
    left_spin();
  } else if (ps2x.Button(PSB_R1)) {
    Serial.println("R1 button pushed");
    right_spin();
  }
}