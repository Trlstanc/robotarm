#include "Settings.h"

//Setup
//Loop
//getSensorInputs
//updateRail
//grabNewBlock
//homeSteppers
//startRecording
//stopRecordingStartReplaying
//recordPosition
//replayRecordedMoves
//dumpBlocks
//returnBlock
//isResetting

#define STEP_COUNT 100
#define SensorAverage 1  

int GrabDelay = 300;
//pickup positions starting point, will be overwritten
long pickupRailPos = -60;       // Rail position for pickup
long pickupLowPos = 4229;    // Default shoulder position for pickup
long pickupHighPos = 5815;   // Default elbow position for pickup

bool hasBlock = false; //tracks if a block needs to be dropped 

void setup() {

  //Serial.begin(115200);

  // Pins for ramps inputs
  pinMode(58, INPUT_PULLUP);
  pinMode(57, INPUT_PULLUP);


  //Pin for recording
  pinMode(RECORD_SWITCH_PIN, INPUT_PULLUP);

  //Pin for Reset
  pinMode(RESET_SWITCH_PIN, INPUT_PULLUP);

  //grabber button
  pinMode(BUTTON_GRABBER, INPUT_PULLUP);
  grabberServo.attach(SERVO_PIN);
  grabberServo.setSpeed(grabberSpeed);

  //smoothing
  shoulderSensor.begin(SMOOTHED_AVERAGE, SMOOTHING_COUNT);
  elbowSensor.begin(SMOOTHED_AVERAGE, SMOOTHING_COUNT);



  //Stepper pins
  stepperHigh.attach(STEP_PIN_HIGH, DIR_PIN_HIGH);
  stepperLow.attach(STEP_PIN_LOW, DIR_PIN_LOW);
  stepperRail.attach(STEP_PIN_RAIL, DIR_PIN_RAIL);
  stepperConv.attach(STEP_PIN_CONV, DIR_PIN_CONV);

  //set gripper
  grabberServo.write(20);  //open
  delay(100);

  //homing function for high, low, rail and grabber
  // Initially disable steppers so user can position arm manually
  pinMode(ENABLE_PIN_LOW, OUTPUT);
  pinMode(ENABLE_PIN_HIGH, OUTPUT);
  pinMode(ENABLE_PIN_RAIL, OUTPUT);
  pinMode(ENABLE_PIN_CONV, OUTPUT);
  
  digitalWrite(ENABLE_PIN_LOW, HIGH);   // Disable
  digitalWrite(ENABLE_PIN_HIGH, HIGH);  // Disable
  digitalWrite(ENABLE_PIN_RAIL, HIGH);  // Disable
  digitalWrite(ENABLE_PIN_CONV, LOW);   // Enable conveyor



  //conveyor belt moving
  pinMode(ENABLE_PIN_CONV, OUTPUT);
  digitalWrite(ENABLE_PIN_CONV, LOW);
  stepperConv.setSpeed(convSpeed);

  stepperConv.move(50);
  delay(500);
  stepperConv.move(-50);

homeRail();

         // Wait for user to press reset button after positioning
  while (digitalRead(RESET_SWITCH_PIN) == HIGH) {
    delay(100); // Wait for button press
  }



  homeAndCalibrate();

  grabSequence = 1;

  startRecording();
}


void loop() {
  

  // Check the state of the record/replay switch
  if (digitalRead(RECORD_SWITCH_PIN) == LOW) {
    if (isRecording) {
      stopRecordingStartReplaying();
      delay(100);
    }
  }

    // Check for Reset
  if (digitalRead(RESET_SWITCH_PIN) == LOW) {
      resetMoves();
      delay(100);
  }

  //Check if replaying
  if (isReplaying) {
    replayRecordedMoves();
  } else { 
    //Do normal inputs

    //sequence to grab block from converyor
    if (grabSequence == 1) {
      grabNewBlock();
    }

    //Grabber - servo
    if (digitalRead(BUTTON_GRABBER) == 0) {
      grabberServo.write(30);  //open
      grabSequence = 1;
      hasBlock = 0;
      //pause to let grippers open
      delay(500);
      
    } else {
      grabberServo.write(10);  //close
    }

    //record position where block was dropped
    if (isRecording && grabSequence == 1) {
      recordPosition();
    }

    // Update target position 10 times per second
    if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
      getSensorInputs();
      lastUpdateTime = millis();
    }  

    updateRail();
    //steppers
    stepperRail.moveTo(xPos);
    stepperLow.moveTo(lowSteps);
    stepperHigh.moveTo(-highStepsAdj);


    //end of else for recording
  }
}

void getSensorInputs(){
  //get sensor values
    int shoulderRead = analogRead(A5);
    int elbowRead = analogRead(A10);

    //add values to running average
    shoulderSensor.add(shoulderRead);
    elbowSensor.add(elbowRead);

    // Map average sensor values to step ranges
    lowSteps = map(shoulderSensor.get(), 110, 825, 0, 7200);
    highSteps = map(elbowSensor.get(), 932, 242, 0, 7200);

    lowSteps = constrain(lowSteps, 0, 7200);
    highSteps = constrain(highSteps, 0, 7200);

    //Adjust high arm steps for combined motion
    highStepsAdj = highSteps + lowSteps - 3600;

    // limit for arm range of motion
    lowSteps = constrain(lowSteps, max(1900, highStepsAdj - 2200), 6500);
    //prevent arm from hitting platform
    highStepsAdj = min(highStepsAdj, highMax[lowSteps / 10]);

    highStepsAdj = constrain(highStepsAdj, 2700, min(6800, lowSteps + 1600));

/*
    Serial.print("low steps: ");
    Serial.print(lowSteps);
    Serial.print(" high steps: ");
    Serial.println(highStepsAdj);
    Serial.print(" rail: ");
    Serial.println(xPos);
  */ 
}

void updateRail(){
      //move x axis
    if (stepperRail.distanceToGo() < 20) {
      if (digitalRead(58) == 0) {
        xPos = xPos - 10;
      } else if (digitalRead(57) == 0) {
        xPos = xPos + 10;
      }
    }

    xPos = constrain(xPos, -2800, -1000);
}



void grabNewBlock() {
  //Increase stepper speeds
  stepperLow.setSpeed(XYspeed * 1.5);
  stepperHigh.setSpeed(XYspeed * 1.5);
  stepperRail.setSpeed(2000);

  //move back
  stepperLow.moveTo(2700);
  stepperHigh.moveTo(-2700);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
    //Check for replay button while moving arm
    if (digitalRead(RECORD_SWITCH_PIN) == LOW) {
      if (isRecording) {
        stopRecordingStartReplaying();
        delay(100);
        return;
      }
    }
    //Check for reset
    if (digitalRead(RESET_SWITCH_PIN) == LOW) {
        resetMoves();
        return;
    }
  }

  //Go to conveyor and open gripper
  stepperRail.moveTo(-60);
  grabberServo.write(35);
  while (stepperRail.moving() > 0) {
    //Check for replay button while moving rail
    if (digitalRead(RECORD_SWITCH_PIN) == LOW) {
      if (isRecording) {
        stopRecordingStartReplaying();
        delay(100);
        return;
      }
    }
    //Check for reset
    if (digitalRead(RESET_SWITCH_PIN) == LOW) {
        resetMoves();
        return;
    }
    
  }


  stepperLow.moveTo(pickupLowPos);
  stepperHigh.moveTo(pickupHighPos);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(GrabDelay);
  //half close and do a little wiggle
  grabberServo.write(15);
  delay(GrabDelay);
    grabberServo.write(11);
  delay(GrabDelay);


  
 
  stepperLow.moveTo(2700);
  stepperHigh.moveTo(-2700);
  
  //conveyor belt moving
     stepperConv.move(50);
     delay(500);
     stepperConv.move(-530);

  

  xPos = 1000;

  //set speeds to normal
  stepperRail.setSpeed(railSpeed);
  stepperLow.setSpeed(XYspeed);
  stepperHigh.setSpeed(XYspeed);

  hasBlock = 1;

  grabSequence = 0;
}


void homeSteppers() {

  digitalWrite(ENABLE_PIN_HIGH, LOW);
  delay(200);
  // Home High Stepper


  stepperHigh.setSpeed(XYspeed / 2);

  stepperHigh.rotate(1);
  while (digitalRead(ENDSTOP_HIGH) == HIGH) {
  }  //wait for homing

  stepperHigh.stop();
  // Move away from endstop
  stepperHigh.move(-200);
  while (stepperHigh.moving()) {
  }

  // Slow final approach
  stepperHigh.setSpeed(XYspeed / 8);
  stepperHigh.rotate(1);
  while (digitalRead(ENDSTOP_HIGH) == HIGH) {
  }
  stepperHigh.stop();

  // Move to nearest full step
  stepperHigh.move(-100);
  while (stepperHigh.moving()) {}



  stepperHigh.setSpeed(XYspeed);
  stepperHigh.setRampLen(XYrampLength);

  stepperHigh.setZero(2600);
  stepperHigh.moveTo(-3600);
  //High Stepper Done

  // Home Low Stepper
  digitalWrite(ENABLE_PIN_LOW, LOW);
  delay(200);
  stepperLow.setSpeed(XYspeed / 2);  // Faster initial approach
  stepperLow.rotate(-1);
  while (digitalRead(ENDSTOP_LOW) == HIGH) {}
  stepperLow.stop();

  // Move away from endstop
  stepperLow.move(200);
  while (stepperLow.moving()) {}

  // Slow final approach
  stepperLow.setSpeed(XYspeed / 8);
  stepperLow.rotate(-1);
  while (digitalRead(ENDSTOP_LOW) == HIGH) {}
  stepperLow.stop();

  // Move to nearest full step
  stepperLow.move(100);
  while (stepperLow.moving()) {}

  stepperLow.setZero(-1800);
  stepperLow.moveTo(3600);
  stepperLow.setSpeed(XYspeed);
  stepperLow.setRampLen(XYrampLength);

  delay(100);
  // Low Stepper Done

  

  //Rail Stepper Home
  pinMode(ENABLE_PIN_RAIL, OUTPUT);
  digitalWrite(ENABLE_PIN_RAIL, LOW);

  stepperRail.setSpeed(railSpeed_Homing);


  stepperRail.rotate(1);
  while (digitalRead(ENDSTOP_RAIL) == HIGH) {
  }

  stepperRail.stop();
  stepperRail.setZero(0);
  stepperRail.writeSteps(-500);
  stepperRail.setSpeed(railSpeed);
  stepperRail.setRampLen(3);
  //Rail Stepper Done

  //Conveyor belt

  //Rail Stepper Home
  pinMode(ENABLE_PIN_CONV, OUTPUT);
  digitalWrite(ENABLE_PIN_CONV, LOW);

  stepperConv.setSpeed(convSpeed);
  stepperLow.setSpeed(XYspeed);
}

//first homing alsos calibrates pickup point
void homeAndCalibrate() {

  //open gripper
  grabberServo.write(30);  //open
  delay(100);

  pinMode(ENABLE_PIN_LOW, OUTPUT);
  digitalWrite(ENABLE_PIN_LOW, LOW);


// Home High Stepper
  pinMode(ENABLE_PIN_HIGH, OUTPUT);
  digitalWrite(ENABLE_PIN_HIGH, LOW);
  
  delay(200);
  
  stepperHigh.setSpeed(XYspeed / 2);

  stepperHigh.setZero();
  
  stepperHigh.rotate(1);
  while (digitalRead(ENDSTOP_HIGH) == HIGH) {
  }  //wait for homing

  stepperHigh.stop();

  // Move away from endstop
  stepperHigh.move(-200);
  while (stepperHigh.moving()) {
  }

  // Slow final approach
  stepperHigh.setSpeed(XYspeed / 8);
  stepperHigh.rotate(1);
  while (digitalRead(ENDSTOP_HIGH) == HIGH) {
  }
  stepperHigh.stop();

  delay(200);


  // Move to nearest full step
  stepperHigh.move(-100);
  while (stepperHigh.moving()) {}

  pickupHighPos = -(stepperHigh.currentPosition() + 2600);

  stepperHigh.setSpeed(XYspeed);
  stepperHigh.setRampLen(XYrampLength);

  stepperHigh.setZero(2600);
  stepperHigh.moveTo(-3700);

  delay(500);


  //High Stepper Done



  // Home Low Stepper
   stepperLow.setZero();

  stepperLow.setSpeed(XYspeed / 2);  // Faster initial approach
  stepperLow.rotate(-1);
  while (digitalRead(ENDSTOP_LOW) == HIGH) {}
  stepperLow.stop();

  // Move away from endstop
  stepperLow.move(200);
  while (stepperLow.moving()) {}

  // Slow final approach
  stepperLow.setSpeed(XYspeed / 8);
  stepperLow.rotate(-1);
  while (digitalRead(ENDSTOP_LOW) == HIGH) {}
  stepperLow.stop();



  // Move to nearest full step
  stepperLow.move(100);
  while (stepperLow.moving()) {}

      pickupLowPos = -(stepperLow.currentPosition() - 1800);




  stepperLow.setZero(-1800);
  stepperLow.moveTo(3600);
  stepperLow.setSpeed(XYspeed);
  stepperLow.setRampLen(XYrampLength);

  delay(500);
  // Low Stepper Done



}

void homeRail(){
  
  //Rail Home
  pinMode(ENABLE_PIN_RAIL, OUTPUT);
  digitalWrite(ENABLE_PIN_RAIL, LOW);

  stepperRail.setSpeed(railSpeed_Homing);


  stepperRail.rotate(1);
  while (digitalRead(ENDSTOP_RAIL) == HIGH) {
  }

  stepperRail.stop();
  pickupRailPos = -(stepperRail.currentPosition());
  stepperRail.setZero(0);
  stepperRail.writeSteps(-60);
  stepperRail.setSpeed(railSpeed);
  stepperRail.setRampLen(3);
  //Rail Stepper Done

}

void startRecording() {
  isRecording = true;
  positionCount = 0;
}

void stopRecordingStartReplaying() {
  isRecording = false;
  isReplaying = true;
}

void recordPosition() {
  if (positionCount < MAX_POSITIONS) {
    recordedPositions[positionCount][0] = xPos;
    recordedPositions[positionCount][1] = lowSteps;
    recordedPositions[positionCount][2] = highStepsAdj;

    positionCount++;

  } 
}

void replayRecordedMoves() {


 if (hasBlock == 1){
    returnBlock();
 }

  dumpBlocks();


  grabNewBlock();

  for (int i = 0; i < positionCount; i++) {
    // Get out of the way
    stepperLow.moveTo(2800);
    stepperHigh.moveTo(-2800);
    stepperRail.moveTo(recordedPositions[i][0]);

    while (stepperRail.moving() || stepperLow.moving() || stepperHigh.moving()) {
        //Check for reset
        if (digitalRead(RESET_SWITCH_PIN) == LOW) {
          resetMoves();
        return;
        }
    }

    //prepare to lower block
    stepperLow.moveTo(4200);
    stepperHigh.moveTo(-2700);
    while (stepperLow.moving() || stepperHigh.moving()) {
        //Check for reset
        if (digitalRead(RESET_SWITCH_PIN) == LOW) {
          resetMoves();
        return;
        }
    }


    stepperLow.moveTo(recordedPositions[i][1]);
    stepperHigh.moveTo(-recordedPositions[i][2]);


    // Wait for movements to complete
    while (stepperLow.moving() || stepperHigh.moving()) {
        //Check for reset
        if (digitalRead(RESET_SWITCH_PIN) == LOW) {
          resetMoves();
        return;
        }
      delay(10);
    }
    //open grabber
    grabberServo.write(30);
    // Pause between moves
    delay(500);


    grabNewBlock();
  }

  isReplaying = false;
  isRecording = true;

}

void dumpBlocks() {
  //move arms out of the way
  stepperLow.moveTo(2700);
  stepperHigh.moveTo(-2700);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }

  //move rail to end
  stepperRail.moveTo(-2730);
  while (stepperRail.moving() > 0) {
  }

  //open grabber
  grabberServo.write(45);

  //move to handle
  stepperLow.moveTo(4320);
  stepperHigh.moveTo(-6000);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  stepperLow.setSpeed(XYspeed / 2);
  stepperHigh.setSpeed(XYspeed / 2);
  //close grabber
  grabberServo.write(5);
  delay(500);

  //first lift
  stepperLow.moveTo(4040);
  stepperHigh.moveTo(-5200);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(100);

  //second lift
  stepperLow.moveTo(3760);
  stepperHigh.moveTo(-4410);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(100);

  //third lift
  stepperLow.moveTo(4030);
  stepperHigh.moveTo(-3900);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(1000);

  //return to 2nd
  stepperLow.moveTo(3760);
  stepperHigh.moveTo(-4410);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(100);

  //return to 1st
  stepperLow.moveTo(4040);
  stepperHigh.moveTo(-5200);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(100);

  //all the way back down
  stepperLow.moveTo(4300);
  stepperHigh.moveTo(-6000);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }
  delay(100);

  grabberServo.write(35);
  delay(500);

  stepperLow.setSpeed(XYspeed);
  stepperHigh.setSpeed(XYspeed);
}

void returnBlock() {
  stepperLow.setSpeed(XYspeed * 1.5);
  stepperHigh.setSpeed(XYspeed * 1.5);
  stepperRail.setSpeed(2000);

    //move back
  stepperLow.moveTo(4800);
  stepperHigh.moveTo(-2700);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }

  //Go to conveyor
  stepperRail.moveTo(-2700);

  while (stepperRail.moving() > 0) {
  }
  stepperLow.moveTo(6500);
  stepperHigh.moveTo(-3600);
  while (stepperLow.moving() > 0 || stepperHigh.moving() > 0) {
  }

  delay(500);
  grabberServo.write(35);
  delay(500);
}


void resetMoves() {
 
 if (hasBlock == 1){
    returnBlock();
 }

  dumpBlocks();
  // rehome arm, disable high stepper first
  digitalWrite(ENABLE_PIN_HIGH, HIGH);
  homeSteppers();
  grabNewBlock();

  positionCount = 0;

  isReplaying = false;
  isRecording = true;

}