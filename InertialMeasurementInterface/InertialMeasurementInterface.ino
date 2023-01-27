#include "BNO055_support.h"   //Contains the bridge code between the API and Arduino
#include <Wire.h>

struct bno055_t myBNO;
typedef struct bno055_Inertial_data_t{ 
    bno055_accel AccelData;
    bno055_mag   MagData;
    bno055_gyro  GyroData;
 } bno055_Inertial_data_t; // Raw Real and Imaginary values

bno055_Inertial_data_t bno055_Inertial_data;

unsigned long lastTime = 0;

void SerialPrintInertialData(bno055_Inertial_data_t bno055_Inertial_data){
    Serial.print("Time Stamp: ");       //To read out the Time Stamp
    Serial.println(lastTime);

    Serial.print("Acceleration (x): ");  
    //Serial.print(",");        
    Serial.println(float(bno055_Inertial_data.AccelData.x)/100); // Divide to account to convert to float. Based on g = 9.8 m/s^2   
    //Serial.print(",");       

    Serial.print("Acceleration (y): ");       
    Serial.println(float(bno055_Inertial_data.AccelData.y)/100);   
    //Serial.print(",");       


    Serial.print("Acceleration (z): ");       
    Serial.println(float(bno055_Inertial_data.AccelData.z)/100);   
    //Serial.print(",");          


    Serial.print("Magnetometer (x): ");       
    Serial.println(float(bno055_Inertial_data.MagData.x));   

    Serial.print("Magnetometer (y): ");       
    Serial.println(float(bno055_Inertial_data.MagData.y));   

    Serial.print("Magnetometer (z): ");       
    Serial.println(float(bno055_Inertial_data.MagData.z));   

    Serial.print("Gyroscope (x): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.x));   

    Serial.print("Gyroscope (y): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.y));   

    Serial.print("Gyroscope (z): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.z));   

}

void getInertialMeasurements(bno055_Inertial_data_t * bno055_Inertial_data){
    bno055_read_accel_xyz(&(bno055_Inertial_data->AccelData));  //Update Acceleration data into the structure
    bno055_read_gyro_xyz(&(bno055_Inertial_data->GyroData));    //Update Gyroscope data into the structure
    bno055_read_mag_xyz(&(bno055_Inertial_data->MagData));      //Update Magnetometer data into the structure
}

void setup() 
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); 

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}

void loop() //This code is looped forever
{
  
  if ((millis() - lastTime) >= 1000) //To stream at 10 Hz without using additional timers
  {
    lastTime = millis();

    getInertialMeasurements(&bno055_Inertial_data);
   
    SerialPrintInertialData(bno055_Inertial_data);
  }
}
