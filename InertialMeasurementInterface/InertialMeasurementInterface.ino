#include "BNO055_support.h"   //Contains the bridge code between the API and Arduino
#include <Wire.h>
#include "utility/imumaths.h"

struct bno055_t myBNO;
typedef struct bno055_Inertial_data_t{ 
    bno055_accel AccelData;
    bno055_mag   MagData;
    bno055_gyro  GyroData; 
    bno055_euler EulerData; // Divide by 16 to get values in degrees
    bno055_quaternion QuatData;
    
 } bno055_Inertial_data_t; // Raw Real and Imaginary values

bno055_Inertial_data_t bno055_Inertial_data;

void convertRotation(bno055_Inertial_data_t*, double*);

unsigned long lastTime = 0;

// This function is used to generate a csv of the data for debugging.
void rotDebug(bno055_Inertial_data_t bno055_Inertial_data) {
  Serial.print(bno055_Inertial_data.AccelData.x);
  Serial.print(",");
  Serial.print(bno055_Inertial_data.AccelData.y);
  Serial.print(",");
  Serial.print(bno055_Inertial_data.AccelData.z);
  Serial.print(",");
  Serial.print(float(bno055_Inertial_data.QuatData.w));
  Serial.print(",");
  Serial.print(float(bno055_Inertial_data.QuatData.x));
  Serial.print(",");
  Serial.print(float(bno055_Inertial_data.QuatData.y));
  Serial.print(",");
  Serial.print(float(bno055_Inertial_data.QuatData.z));
  Serial.println(",");
}
void SerialPrintInertialData(bno055_Inertial_data_t bno055_Inertial_data){
    Serial.print("Time Stamp: ");       //To read out the Time Stamp
    Serial.println(lastTime);
    
    Serial.print("Acceleration (x): ");        
    Serial.print(float(bno055_Inertial_data.AccelData.x)/100); // Divide to account to convert to float. Based on g = 9.8 m/s^2       
    Serial.print(" (y): ");       
    Serial.print(float(bno055_Inertial_data.AccelData.y)/100);        
    Serial.print(" (z): ");       
    Serial.println(float(bno055_Inertial_data.AccelData.z)/100);       

    /*
    Serial.print("Magnetometer (x): ");       
    Serial.print(float(bno055_Inertial_data.MagData.x));   
    Serial.print(" (y): ");       
    Serial.print(float(bno055_Inertial_data.MagData.y));   
    Serial.print(" (z): ");       
    Serial.println(float(bno055_Inertial_data.MagData.z));   
    /*
    Serial.print("Gyroscope (x): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.x));   

    Serial.print("Gyroscope (y): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.y));   

    Serial.print("Gyroscope (z): ");       
    Serial.println(float(bno055_Inertial_data.GyroData.z));   
  */
    Serial.print("Euler  Angles: Heading: ");
    Serial.print(float(bno055_Inertial_data.EulerData.h) / 16.0);
    Serial.print(" Roll: ");
    Serial.print(float(bno055_Inertial_data.EulerData.r)  / 16.0);
    Serial.print(" Pitch: ");
    Serial.println(float(bno055_Inertial_data.EulerData.p)  / 16.0);

    Serial.print("Quaternion: w: ");
    Serial.print(float(bno055_Inertial_data.QuatData.w));
    Serial.print(" x: ");  
    Serial.print(float(bno055_Inertial_data.QuatData.x));
    Serial.print(" y: ");  
    Serial.print(float(bno055_Inertial_data.QuatData.y));
    Serial.print(" z: ");  
    Serial.println(float(bno055_Inertial_data.QuatData.z));

    
    double newAccel[3] = {0.0, 0.0, 0.0};
    convertRotation(&bno055_Inertial_data, newAccel);
    Serial.print("Constant Accel: x: ");
    Serial.print(newAccel[0]);
    Serial.print(" y: ");
    Serial.print(newAccel[1]);
    Serial.print(" z: ");
    Serial.println(newAccel[2]);
    
    

}

void getInertialMeasurements(bno055_Inertial_data_t * bno055_Inertial_data){
    static double quat_scale = (1.0 / (1 << 14));
    bno055_read_accel_xyz(&(bno055_Inertial_data->AccelData));  //Update Acceleration data into the structure
    bno055_read_gyro_xyz(&(bno055_Inertial_data->GyroData));    //Update Gyroscope data into the structure
    bno055_read_mag_xyz(&(bno055_Inertial_data->MagData));      //Update Magnetometer data into the structure
    bno055_read_euler_hrp(&(bno055_Inertial_data->EulerData));
    bno055_read_quaternion_wxyz(&(bno055_Inertial_data->QuatData));
    //bno055_Inertial_data->QuatData.w = bno055_Inertial_data->QuatData.w * quat_scale;
    //bno055_Inertial_data->QuatData.x = bno055_Inertial_data->QuatData.x * quat_scale;
    //bno055_Inertial_data->QuatData.y = bno055_Inertial_data->QuatData.y * quat_scale;
    //bno055_Inertial_data->QuatData.z = bno055_Inertial_data->QuatData.z * quat_scale;
    
}
double dotProduct(int n, double* a, double* b) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
      sum = sum + (a[i]*b[i]);
  }  
  return sum;
}
void convertRotation(bno055_Inertial_data_t* bno, double* returnAccel) {
  static double quat_scale = (1.0 / (1 << 14));
  // This function DOES NOT work. It attempts to use a rotation matrix to undo the rotation.
  double heading = ((double)bno->EulerData.h / 16.0) * 0.017453; // Convert to radians
  double roll = ((double)bno->EulerData.r  / 16.0) * 0.017453;
  double pitch = ((double)bno->EulerData.p  / 16.0) * 0.017453;

  double ax = (double)bno->AccelData.x / 100.0;
  double ay = (double)bno->AccelData.y / 100.0;
  double az = (double)bno->AccelData.z / 100.0;

    //multiplication of linear acceleration and quaternion
  float tempQuat[4];

  imu::Vector<3> euler = imu::Vector<3>(ax, ay, az);
  imu::Quaternion quat = imu::Quaternion((double)bno->QuatData.w * quat_scale,
                                        (double)bno->QuatData.y  * quat_scale,
                                        (double)bno->QuatData.x  * quat_scale,
                                        (double)bno->QuatData.z  * quat_scale);
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //imu::Quaternion quat = bno.getQuat();

  //http://es.mathworks.com/help/aeroblks/quaternionmultiplication.html   q=quat, r=linAcc
  tempQuat[0] = 0 - euler.x() * quat.x() - euler.y() * quat.y() - euler.z() * quat.z();
  //Serial.print("tempquat[0]->");
  //Serial.println(tempQuat[0]);
  tempQuat[1] = 0 + euler.x() * quat.w() - euler.y() * quat.z() + euler.z() * quat.y();
  tempQuat[2] = 0 + euler.x() * quat.z() + euler.y() * quat.w() - euler.z() * quat.x();
  tempQuat[3] = 0 - euler.x() * quat.y() + euler.y() * quat.x() + euler.z() * quat.w();

  //q=tempQuat, r=quatConj
  returnAccel[0] = quat.w() * tempQuat[1] - quat.x() * tempQuat[0] + quat.y() * tempQuat[3] - quat.z() * tempQuat[2];
  returnAccel[1] = quat.w() * tempQuat[2] + quat.x() * tempQuat[3] - quat.y() * tempQuat[0] + quat.z() * tempQuat[1];
  returnAccel[2] = quat.w() * tempQuat[3] + quat.x() * tempQuat[2] - quat.y() * tempQuat[1] - quat.z() * tempQuat[0];
  Serial.println(quat.w());
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
    //rotDebug(bno055_Inertial_data);
    SerialPrintInertialData(bno055_Inertial_data);
  }
}
