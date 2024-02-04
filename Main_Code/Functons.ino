void setupUp_to_Mile_Stone_1() {
  Serial.begin(57600);
  Serial.print("Starting up Robot code...... ");

  // Run setup code
  setupRSLK();
  myservo.attach(SRV_0);  // This is pin 38, from RSLK_Pins.h
  if (CurrentRemoteMode == 0) {
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
  } else if (CurrentRemoteMode == 1) {
    // put start-up code for IR controller here if neccessary
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
}
void control() {
  // stop();
  Serial.print("Chosing Controler");
  // Read input from PlayStation controller
  ps2x.read_gamepad();

  // Operate the robot in remote control mode
  if (CurrentRemoteMode == 0) {
    Serial.println("Running remote control with the Playstation Controller");
    RemoteControlPlaystation();

  } else if (CurrentRemoteMode == 1) {
    // put code here to run using the IR controller if neccessary
    Serial.println("Running remote control with the IR Controller");
    IRControler();
  }
}
void forward() {
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, 20);
  setMotorSpeed(RIGHT_MOTOR, 21);
}
/* Moves robot backwards: both motors forward same speed */
void backwards() {
  Serial.println("Backwards");
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, fastSpeed);
}
/* Moves robot spin right: both motors forward same speed */
void right_spin() {
  Serial.println("Right Spin");
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
  Serial.println("Open");
  for (int pos = gripper_pos; pos < 91; pos += 1)  
  {                                                 // in steps of 1 degree
    myservo.write(pos);                             // tell servo to go to position in variable 'pos'
    delay(15);                                      // waits 15 ms for the servo to reach the position
  }
  gripper_pos = 90;
}
/* Closes the gripper */
void close() {
  for (int pos = gripper_pos; pos >= 41; pos -= 1) {  
    myservo.write(pos);                               // tell servo to go to position in variable 'pos'
    delay(15);                                        // waits 15 ms for the servo to reach the position
  }
  gripper_pos = 41;
}
/* Stops robot forward: both motors disabled */
void stop() {
  Serial.println("Stop");
  disableMotor(BOTH_MOTORS);
}
void turnNinetyRight() {
  right_turn();
  delay(20);
  stop();
}
void autonomous() {
  Serial.println("Running Autonomus Mode");
  forward_until_bump();
  backwards();
  delay(500);
  right_spin();
  delay(655);
  stop();
  forward();
  lineFollowing();
  backwards();
  delay(2000);
  right_spin();
  delay(1000);
  stop();
  close();
  control();
}
void transmit() {
  sendIR.write(&IRmsg);
  delay(1000);
}
void setup_transmit() {
    // Serial.begin(57600);
    delay(500); // To be able to connect Serial monitor after reset or power up 
    Serial.println(F("START " __FILE__ " from " __DATE__));
    /*
     * Must be called to initialize and set up IR transmit pin.
     *  bool IRsender::initIRSender( )
     */
    if (sendIR.initIRSender()) {
        Serial.println(F("Ready to transmit NEC IR signals on pin " STR(IR_TRX_PIN)));
    } else {
        Serial.println("Initialization of IR transmitter failed!");
        while (1) {;}
    }
    delay(200);
    // enable transmit feedback and specify LED pin number (defaults to LED_BUILTIN)
    enableTXLEDFeedback(BLUE_LED);

    IRmsg.protocol = NEC;
    IRmsg.address = 0xEE;
    IRmsg.command = 0xA0;
    IRmsg.isRepeat = false;
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
  } else if (ps2x.Button(PSB_CIRCLE)) {
    Serial.println("CIRCLE button pushed");
    autonomous();
  } else if (ps2x.Button(PSB_START)) {
    Serial.println("START button pushed");
    transmit();
  }
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
      case 0x45:
        Serial.println("CIRCLE button pushed");
        autonomous();
        break;
     case 0xD:
        Serial.println("ST button pushed");
        transmit();
        break; 
    }
  }
}
void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  UART_SERIAL.println("Push left button on Launchpad to begin calibration.");
  UART_SERIAL.println("Make sure the robot is on the floor away from the line.\n");
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, RED_LED);

  delay(500);
  UART_SERIAL.println("Running calibration on floor");

  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS, 20);

  /* Must be called prior to using getLinePosition() or readCalLineSensor() */
  calibrateLineSensor(lineColor);

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);

  UART_SERIAL.println("Reading floor values complete");

  UART_SERIAL.println("Push left button on Launchpad to begin line following.");
  UART_SERIAL.println("Make sure the robot is on the line.\n");
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN, RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}
void lineFollowing() {

  Serial.println("Running Line Following");
  bool hitObstacle = false;
  while (!hitObstacle) {
    uint32_t linePos = getLinePosition();
    Serial.println("Testing line Position");
    if ((linePos > 0) && (linePos < 4000)) {  // turn left
      setMotorSpeed(LEFT_MOTOR, 8);
      setMotorSpeed(RIGHT_MOTOR, 13);
    } else if (linePos > 5000) {  // turn right
      setMotorSpeed(LEFT_MOTOR, 13);
      setMotorSpeed(RIGHT_MOTOR, 8);
    } else {  // go straight
      setMotorSpeed(LEFT_MOTOR, 12);
      setMotorSpeed(RIGHT_MOTOR, 12);
    }

    /* Check if any bump switch was pressed */
    if (getBumpSwitchPressed() > 0) {
      hitObstacle = true;
      break;
    }
  }

  Serial.println("Collision detected\n");
  disableMotor(BOTH_MOTORS);
  delay(100);
}
void setup_Bump_Switches_and_lineFollowing() {
  Serial.println("setup bump");
  setupRSLK();
  if (isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }
}
void forward_until_bump() {
  bool hitObstacle = false;

  /* Wait two seconds before starting */
  delay(2000);
  Serial.print("turning on motors");
  /* Enable both motors, set their direction and provide a default speed */
  // forward();
  disableMotor(BOTH_MOTORS);
  delay(50);
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, 20);
  setMotorSpeed(RIGHT_MOTOR, 21);

  /* Keep checking if the robot has hit an object */
  while (!hitObstacle) {
    /* Check if any bump switch was pressed */
    if (getBumpSwitchPressed() > 0) {
      hitObstacle = true;
      break;
    }
  }

  Serial.println("Collision detected\n");
  disableMotor(BOTH_MOTORS);
}
