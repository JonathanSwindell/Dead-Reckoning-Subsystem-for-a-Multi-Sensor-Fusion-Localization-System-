#include "BNO055.h"
#include <Wire.h>
#define I2C_ADDR 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH

//+Y is North
//-Y is South
//+X is East
//-X is West

// Global variables
BNO055 myBNO(I2C_ADDR); // Object for the BNO sensor
String inputCommand = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// Configuration stuff
#define BAUD_RATE 115200
#define SERIAL_RATE_DELAY 10 // In ms // 100Hz Sampling Frequency
#define MODE_DEBUG

void setup(){
    Wire.begin();
    Serial.begin(BAUD_RATE);
    myBNO.init();
}


void loop(){
    static unsigned long lastTime = 0;

    //Read input from PC
    if (stringComplete) {
      Serial.print("Recived input: "); Serial.print(inputCommand);
      if (inputCommand == "r") {
        Serial.println("Resetting Dead Reckoning!");
        myBNO.resetDeadReckoning();
      }
      // clear the string:
      inputCommand = "";
      stringComplete = false;
    }
    
    //Dead Reckoning
    myBNO.deadReckoning(0);   //Uses local coordinates, x component of acceleration will be in the sensor's x axis

    myBNO.readGyro();
    myBNO.readMag();
    myBNO.readQuat();
    myBNO.readEul();
    myBNO.readLinAcc();
    myBNO.readAbsAcc();



    myBNO.readCalibrationStatus();
    /*
    Serial.print("Calibration\nSystem Status ");
    Serial.println(myBNO.sysSta);

    Serial.print("gyro Status ");
    Serial.println(myBNO.gyroSta);

    Serial.print("accelSta Status ");
    Serial.println(myBNO.accelSta);

    Serial.print("magSta Status ");
    Serial.println(myBNO.magSta);
  */
    //Add some control

    if ((millis()- lastTime) > SERIAL_RATE_DELAY) {
      lastTime = millis();
      Serial.print("C:"); Serial.print(myBNO.sysSta); Serial.print(", "); Serial.print(myBNO.gyroSta);  Serial.print(", ");Serial.print(myBNO.accelSta); Serial.print(", "); Serial.println(myBNO.magSta);
      Serial.print("Lo:"); Serial.print(myBNO.position.x); Serial.print(", "); Serial.print(myBNO.position.y); Serial.print(", "); Serial.println(myBNO.position.z);
      Serial.print("M:"); Serial.print(myBNO.mag.x); Serial.print(", "); Serial.print(myBNO.mag.y); Serial.print(", "); Serial.println(myBNO.mag.z);
      Serial.print("G:"); Serial.print(myBNO.gyro.x); Serial.print(", "); Serial.print(myBNO.gyro.y); Serial.print(", "); Serial.println(myBNO.gyro.z);
      Serial.print("Q:"); Serial.print(myBNO.quat.q0); Serial.print(", "); Serial.print(myBNO.quat.q1); Serial.print(", "); Serial.print(myBNO.quat.q2); Serial.print(", "); Serial.println(myBNO.quat.q3);
      Serial.print("E:"); Serial.print(myBNO.euler.x); Serial.print(", "); Serial.print(myBNO.euler.y); Serial.print(", "); Serial.println(myBNO.euler.z);
      Serial.print("La:"); Serial.print(myBNO.linAcc.x*0.00981); Serial.print(", "); Serial.print(myBNO.linAcc.y*0.00981); Serial.print(", "); Serial.println(myBNO.linAcc.z*0.00981);
      Serial.print("Aa:"); Serial.print(myBNO.absAccel.x*0.00981); Serial.print(", "); Serial.print(myBNO.absAccel.y*0.00981); Serial.print(", "); Serial.println(myBNO.absAccel.z*0.00981);

      
    }
}

void serialEvent() {
  Serial.print("in serial event.");
  
  while (Serial.available()) {
      Serial.print("in serial while.");

    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputCommand += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
  Serial.print((int)inChar);
    
    if (inChar == 'r') {
      stringComplete = true;
    }
  }
}
