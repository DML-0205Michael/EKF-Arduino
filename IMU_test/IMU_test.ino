#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A


#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
Adafruit_MPU6050 mpu;
// #include <EEPROM.h>
// #define EEPROM_size 24 // ax, ay, az error, gx,gy,gz error

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s
Kalman kalmanX; // roll
Kalman kalmanY; // pitch
double p_kf=0, r_kf;
uint32_t timer_kf;


void setup() {
    // put your setup code here, to run once:
    Serial.begin(460800);
    while (!Serial);
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    // IMU_calibration(); 

    digitalWrite(LED_RED,LOW); delay(100);digitalWrite(LED_RED,HIGH);delay(100);
    digitalWrite(LED_RED,LOW); delay(100);digitalWrite(LED_RED,HIGH);delay(100);
    digitalWrite(LED_RED,LOW); delay(100);digitalWrite(LED_RED,HIGH);delay(100);
    digitalWrite(LED_RED,LOW); delay(100);digitalWrite(LED_RED,HIGH);delay(100);
    delay(1000);
}

unsigned long pre_time=0;
void loop() {
  // int dt=micros()-pre_time;
  // Serial.print("dt=");Serial.print(dt); Serial.print("\t");
  // pre_time=micros();
  int p1=analogRead(A2);
  int p2=analogRead(A4);
  int diff=p2-p1;
  Serial.print(diff);
  if (diff>10) {Serial.print("Pushed");}
  else Serial.print("No Push");
  

  read_accel_gyro_raw();
  //Accelerometer
  Serial.print("ax="); Serial.print(ax_float, 2); Serial.print("\t");
  Serial.print("ay="); Serial.print(ay_float, 2); Serial.print("\t");
  Serial.print("az="); Serial.print(az_float, 2); Serial.print("\t");

  //Gyroscope
  Serial.print("gx="); Serial.print(wx, 2); Serial.print("\t");
  Serial.print("gy="); Serial.print(wy, 2); Serial.print("\t");
  Serial.print("gz="); Serial.print(wz, 2); Serial.print("\t");

    //Thermometer
    // Serial.print("\nThermometer:\n");
    // Serial.print(" Degrees C1 = ");
    // Serial.println(myIMU.readTempC(), 4);
    // Serial.print(" Degrees F1 = ");
    // Serial.println(myIMU.readTempF(), 4);

    Serial.println();
}

// void read_angle_loop(){
//   read_accel_gyro_raw();
//   ax_float-=ax_error;
//   ay_float-=ay_error;
//   az_float-=az_error;
//   wx-=wx_error;
//   wy-=wy_error;
//   wz-=wz_error;

//   double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
//   timer_kf = micros();
  
//   double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); // *180/M_PI; 
//   double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float))); // *180/M_PI;

//   // double roll_ang_v = wx + (sin(p_kf)*sin(r_kf)/cos(p_kf))*wy + (sin(p_kf)*cos(r_kf)/cos(p_kf))*wz;// roll angular velocity
//   // double pitch_ang_v = cos(r_kf)*wy - sin(r_kf)*wz;// pitch angular velocity, rad/s

//   r_kf = kalmanX.getAngle(roll, wy, dt); // roll, rad
//   p_kf = kalmanY.getAngle(pitch, wx, dt); // pitch, rad

//   float yaw_v = (wz - wx * sin(p_kf) + wy * cos(p_kf) * sin(r_kf)) / (cos(r_kf) * cos(p_kf));

  
//   // Serial.print("p_kf:");Serial.print(actual[1][0]); Serial.print("\t");
//   // Serial.print("r_kf:");Serial.print(actual[0][0]); Serial.print("\t");
//   // Serial.print("wx:");Serial.print(actual[1][3]); Serial.print("\t");
//   // Serial.print("wy:");Serial.print(actual[0][3]); Serial.print("\t");
//   // Serial.print("Yaw rate:");Serial.print(actual[3][3]); Serial.print("\t");
//   // Serial.print("p ang v:");Serial.print(pitch_ang_v*180/M_PI); Serial.print("\t");
// }

void read_accel_gyro_raw(){
  ax_float=myIMU.readFloatAccelX();
  ay_float=myIMU.readFloatAccelY();
  az_float=myIMU.readFloatAccelZ();
  wx=myIMU.readFloatGyroX();
  wy=myIMU.readFloatGyroY();
  wz=myIMU.readFloatGyroZ();

}

// void IMU_calibration() {
//   digitalWrite(LED_RED,LOW);
//   digitalWrite(LED_BLUE,LOW);
//   Serial.println("Start IMU calibration. ");
//   // acceleration error
//   int num_of_loop=1000; // number of sample
//   int i=0;
//   while (i < num_of_loop) {
//     read_accel_gyro_raw();
//     ax_error=ax_error+ax_float; // unit: g
//     ay_error=ay_error+ay_float;
//     az_error=az_error+az_float;
//     i++;
//   }
//   ax_error=ax_error/num_of_loop; // unit: raw data
//   ay_error=ay_error/num_of_loop;
//   az_error=(az_error/num_of_loop-9.81); // should measure 1g when stationary. chip face is -z
  
//   // angular velocity error
//   i=0;
//   while (i < num_of_loop) {
//     read_accel_gyro_raw();
//     wx_error=wx_error+wx; // unit: rad/sec
//     wy_error=wy_error+wy;
//     wz_error=wz_error+wz;
//     i++;
//   }
//   wx_error=wx_error/num_of_loop; // rad/sec
//   wy_error=wy_error/num_of_loop;
//   wz_error=wz_error/num_of_loop;

//   delay(500);

//   Serial.println("Acceleration error (g): ");
//   Serial.print("ax_error: ");
//   Serial.println(ax_error);
//   Serial.print("ay_error: ");
//   Serial.println(ay_error);
//   Serial.print("az_error: ");
//   Serial.println(az_error);
//   Serial.println();

//   Serial.println("Angular velocity error (rad/s): ");
//   Serial.print("wx_error: ");
//   Serial.println(wx_error);
//   Serial.print("wy_error: ");
//   Serial.println(wy_error);
//   Serial.print("wz_error: ");
//   Serial.println(wz_error);

//   Serial.println("Accel and gyro Calibration finished.");
//   Serial.println();
//   delay(1000);
//   digitalWrite(LED_RED,HIGH);
//   digitalWrite(LED_BLUE,HIGH);
// }
