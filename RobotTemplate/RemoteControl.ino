/*
  RemoteControl.ino - Arduino Sketch for Remote Control of the RLSK robot
  
  Description:
  This sketch provides functions for controlling an RLSK robot using a
  playstation controller for input. A few actions are programed for an 
  example. 

  Button map:
  PAD UP button moves both motors forward
  CROSS button stops motors

  Created by: Your Name
  Date: Current Date
  Version: 1.0
*/

void RemoteControl() {
  // put your code here to run in remote control mode

// Receive and decode remote control commands
    if(ps2x.Button(PSB_PAD_UP)) {      
      Serial.print("PAD UP button pushed ");
      forward();
    }
    else if(ps2x.Button(PSB_CROSS)){
      Serial.print("CROSS button pushed");
      stop();
    }   
}
