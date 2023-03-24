#include "BNO055.h"
#include <Wire.h>
#define I2C_ADDR 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH

//+Y is North
//-Y is South
//+X is East
//-X is West

//TODO 
//Add calibration status bits

// Global variables
BNO055 myBNO(I2C_ADDR); // Object for the BNO sensor
String inputCommand = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// Configuration stuff
#define BAUD_RATE 9600
#define SERIAL_RATE_DELAY 1000 // In ms
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

    //Atherton - Is there a reason this is done twice?
    
    myBNO.deadReckoning(0);   //Uses local coordinates, x component of acceleration will be in the sensor's x axis

    myBNO.readGyro();
    myBNO.readMag();

    //Serial.print(myBNO.readByte(I2C_ADDR,BNO055_CALIB_STAT);


    if ((millis()- lastTime) > SERIAL_RATE_DELAY) {
      lastTime = millis();
      Serial.print("Location(X,Y,Z): "); Serial.print(myBNO.position.x); Serial.print(" , "); Serial.print(myBNO.position.y); Serial.print(" , "); Serial.print(myBNO.position.z); Serial.print(" , ");
      Serial.print("Magnetic Feild Strength (X,Y,Z): "); Serial.print(myBNO.mag.x); Serial.print(" , "); Serial.print(myBNO.mag.y); Serial.print(" , "); Serial.print(myBNO.mag.z); Serial.print(" , ");
      Serial.print("Gyroscope (X,Y,Z): "); Serial.print(myBNO.gyro.x); Serial.print(" , "); Serial.print(myBNO.gyro.y); Serial.print(" , "); Serial.print(myBNO.gyro.z);
      Serial.print("\n");
      
      
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
