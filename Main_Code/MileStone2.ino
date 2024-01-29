// Jocelyn, Ian, Owen
// 1/23/24
//The program causes the robot to move baced on the ps and ir controlers
// The harware used was the PS and IR controlers and recivers and the robot
// there are funtions for each of the movements and functions to relate butons to the movement funtions 
// 
// Load libraries used
#include "SimpleRSLK.h"
#include "Servo.h"
#include "PS2X_lib.h"
#include <TinyIRremote.h>
#include <SharpDistSensor.h>
// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
/* Modify the following line to use an alternate UART interface (i.e. Serial1/2/3) */
#define UART_SERIAL     Serial
// Define pin numbers for the button on the PlayStation controller
#define PS2_DAT 14  //P1.7 <-> brown wire
#define PS2_CMD 15  //P1.6 <-> orange wire
#define PS2_SEL 34   //P2.3 <-> yellow wire (also called attention)
#define PS2_CLK 35   //P6.7 <-> blue wire
// Define pin numbers for the button on the  IR controler
#define sensorPin 26 //P4.4 <-> ???????
#define IR_RCV_PIN 19  // P2.5 <-> black wire
IRreceiver irRX(IR_RCV_PIN);

// Create an instance of the playstation controller object
PS2X ps2x;
IRData IRresults;
// Define remote mode either playstation controller or IR remote controller
enum RemoteMode {
  PLAYSTATION,
  IR_REMOTE
};

// Declare and initialize the current state variable
RemoteMode CurrentRemoteMode = PLAYSTATION;

// Tuning Parameters
const uint16_t lowSpeed = 10;
const uint16_t normalSpeed = 15;
const uint16_t fastSpeed = 20;

// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;

/* Valid values are either:
 *  DARK_LINE  if your floor is lighter than your line
 *  LIGHT_LINE if your floor is darker than your line
 */
const uint8_t lineColor = LIGHT_LINE;

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

// Defining Distances 
int stopDistance = 10;
int error;
int motorSpeed;

Servo myservo;          // create servo object to control a servo
int gripper_pos = 139;  // Gripper starts open
bool isCalibrationComplete = false; // Setup Calabration
void setup(){ // all setup for milestone 1 
  // Set up line flollowing
  // UART_SERIAL.begin(115200); // causes bugs 

    setupRSLK();
    /* Left button on Launchpad */
    setupWaitBtn(LP_LEFT_BTN);
    /* Red led in rgb led */
    setupLed(RED_LED);

    setupUp_to_Mile_Stone_1();
    setup_Bump_Switches();
}
void loop(){
    control();       //////////////////////////////////////////////This what you change to unit test ///////////////////////////////
}
