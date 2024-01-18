// Load libraries used
#include "SimpleRSLK.h"
#include "Servo.h"
#include "PS2X_lib.h"
#include <TinyIRremote.h>

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// Define pin numbers for the button on the  IR controler
#define IR_RCV_PIN 33  // P5.1 <-> black wire
IRreceiver irRX(IR_RCV_PIN);

// Create an instance of the playstation and IR controller object
PS2X ps2x;
IRData IRresults;
// Define remote mode IR controler
enum RemoteMode {
  IR_REMOTE,
};

// Declare and initialize the current state variable
// RemoteMode CurrentRemoteMode = IR_REMOTE;

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

  Serial.println(F("START " __FILE__ " from " __DATE__));
  /*
     * Must be called to initialize and set up IR receiver pin.
     *  bool initIRReceiver(bool includeRepeats = true, bool enableCallback = false,
                void (*callbackFunction)(uint16_t , uint8_t , bool) = NULL)
     */
  if (irRX.initIRReceiver()) {
    Serial.println(F("Ready to receive NEC IR signals at pin " STR(IR_RCV_PIN)));
  } else {
    Serial.println("Initialization of IR receiver failed!");
    while (1) { ; }
  }
  // enable receive feedback and specify LED pin number (defaults to LED_BUILTIN)
  enableRXLEDFeedback(BLUE_LED);
}

void loop() {
  // Operate the robot in remote control mode
  Serial.println("Running remote control with the IR Controller");
  IRControler();
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
  gripper_pos = 139;
}
/* Closes the gripper */
void close() {
  for (int pos = gripper_pos; pos >= 41; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                               // tell servo to go to position in variable 'pos'
    delay(15);                                        // waits 15 ms for the servo to reach the position
  }
  gripper_pos = 41;
}
/* Stops robot forward: both motors disabled */
void stop() {
  disableMotor(BOTH_MOTORS);
}

void IRControler() {
  if (irRX.decodeIR(&IRresults)) {
    switch (IRresults.command) {
      case 0x18:
        Serial.println("2 button pushed ");
        forward();
        break;
      case 0x52:
        Serial.println("8 button pushed ");
        backwards();
        break;
      case 0x1C:
        Serial.println("5 button pushed");
        stop();
        break;
      case 0x16:
        Serial.println("0 button pushed");
        open();
        break;
      case 0x19:
        Serial.println("EQ button pushed");
        close();
        break;
      case 0xC:
        Serial.println("1 button pushed");
        left_turn();
        break;
      case 0x5E:
        Serial.println("3 button pushed");
        right_turn();
        break;
      case 0x8:
        Serial.println("4 button pushed");
        left_spin();
        break;
      case 0x5A:
        Serial.println("6 button pushed");
        right_spin();
        break;
    }
  }
}