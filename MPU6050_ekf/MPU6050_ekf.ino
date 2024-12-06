/*
https://robotics.stackexchange.com/questions/23179/sensor-fusion-with-extended-kalman-filter-for-roll-and-pitch
About this link:
1. Wrong covarance matrix P equation
2. C31 should be positive not negative
3. The sign of cos is positive in A12

Correct Equation:
https://ahrs.readthedocs.io/en/latest/filters/ekf.html
*/
////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
//       ^^^^^^^^^^ not ekf

Adafruit_MPU6050 mpu;

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s

double p_kf=0,r_kf=0;
uint32_t timer_kf;
float yaw_v=0, yaw=0;
const float dt=0.005;
float P[2][2]={{0,0},{0,0}};
float Q[2][2]={{0.003,0},{0,0.003}};
float R[3][3]={{0.01,0,0},{0,0.01,0},{0,0,0.01}};
float K[2][3]={{0,0,0},{0,0,0}};
const float g=9.81;
////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#define red_LED_pin 15
#define green_LED_pin 14

void setup() {
  Serial.begin(460800);
  pinMode(red_LED_pin,OUTPUT);
  pinMode(green_LED_pin,OUTPUT);

  mpu6050_setup();

  digitalWrite(red_LED_pin,HIGH); delay(100); digitalWrite(red_LED_pin,LOW); delay(100);
  digitalWrite(red_LED_pin,HIGH); delay(100); digitalWrite(red_LED_pin,LOW); delay(100);
  digitalWrite(red_LED_pin,HIGH); delay(100); digitalWrite(red_LED_pin,LOW); delay(100);
  digitalWrite(red_LED_pin,HIGH); delay(100); digitalWrite(red_LED_pin,LOW); delay(100);
  digitalWrite(green_LED_pin,HIGH);
  Serial.print("Set up ends");

}

void loop() {  
  loop_time_holder();
  read_mpu6050_angle_loop();
  Serial.println();
}

////////////////////////////////////////////// TIME //////////////////////////////////////////////
unsigned long loop_start_time=0;
const int loop_time=5000; // micro seconds
void loop_time_holder(){
  if(micros() - loop_start_time > (loop_time+50)) digitalWrite(red_LED_pin, HIGH);
  else digitalWrite(red_LED_pin, LOW);

  // unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  loop_start_time = micros(); 
}

////////////////////////////////////////////// TIME //////////////////////////////////////////////
////////////////////////////////////////////// IMU //////////////////////////////////////////////
void mpu6050_setup(){ 
  Serial.flush();

  delay(1000);
  // Serial.println("wait for serial port input");
  // delay(5000); // wait for serial port input
  

  mpu6050_start();
  delay(1000);
  IMU_calibration();
  delay(1000);

  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;

  // p_kf=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); 
  // r_kf=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));
  p_kf=0;
  r_kf=0;

  // double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); 
  // double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));
  // kalmanX.setAngle(roll); // Set starting angle
  // kalmanY.setAngle(pitch);
  timer_kf = micros();
}

void mpu6050_start(){
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void read_accel_gyro_raw(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax_float=a.acceleration.x;
  ay_float=a.acceleration.y;
  az_float=a.acceleration.z;
  wx=g.gyro.x;
  wy=g.gyro.y;
  wz=g.gyro.z;
}

void IMU_calibration() {
  digitalWrite(red_LED_pin,HIGH); 
  Serial.println("Start IMU calibration. ");
  // acceleration error
  int num_of_loop=1000; // number of sample
  int i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    ax_error=ax_error+ax_float; // unit: m/s^2
    ay_error=ay_error+ay_float;
    az_error=az_error+az_float;
    i++;
  }
  ax_error=ax_error/num_of_loop; // unit: raw data
  ay_error=ay_error/num_of_loop;
  az_error=(az_error/num_of_loop-9.81); // should measure 1g when stationary. chip face is -z
  // EEPROM.put(0, ax_error);
  // EEPROM.put(4, ay_error);
  // EEPROM.put(8, az_error);
  
  // angular velocity error
  i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    wx_error=wx_error+wx; // unit: rad/sec
    wy_error=wy_error+wy;
    wz_error=wz_error+wz;
    i++;
  }
  wx_error=wx_error/num_of_loop; // rad/sec
  wy_error=wy_error/num_of_loop;
  wz_error=wz_error/num_of_loop;
  // EEPROM.put(12, wx_error);
  // EEPROM.put(16, wy_error);
  // EEPROM.put(20, wz_error);
  // EEPROM.commit();
  delay(500);
  Serial.println("EEPROM wrote");
  delay(2000);

  Serial.println("Minus these errors when converting from int16_t to float. ");
  Serial.println("Acceleration error (m/s^2): ");
  Serial.print("ax_error: ");
  Serial.println(ax_error);
  Serial.print("ay_error: ");
  Serial.println(ay_error);
  Serial.print("az_error: ");
  Serial.println(az_error);
  Serial.println();

  Serial.println("Angular velocity error (rad/s): ");
  Serial.print("wx_error: ");
  Serial.println(wx_error);
  Serial.print("wy_error: ");
  Serial.println(wy_error);
  Serial.print("wz_error: ");
  Serial.println(wz_error);

  Serial.println("Accel and gyro Calibration finished.");
  Serial.println();
  digitalWrite(red_LED_pin,LOW); delay(100);
}

void read_mpu6050_angle_loop(){
  
  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;
  wx-=wx_error;
  wy-=wy_error;
  wz-=wz_error;
  

  // double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
  // timer_kf = micros();
  
  // double pitch=atan2f(ay_float,sqrt(sq(ax_float)+sq(az_float))); // *180/M_PI; 
  // double roll=atan2f(-1*ax_float,sqrt(sq(ay_float)+sq(az_float))); // *180/M_PI;
  // double yaw=atan(sqrt(sq(ax_float)+sq(ay_float))/az_float);
  // yaw_v = (wz - wx * sin(p_kf) + wy * cos(p_kf) * sin(r_kf)) / (cos(r_kf) * cos(p_kf));
  // Serial.print("wx:"); Serial.print(wx); Serial.print("\t");
  // Serial.print("wy:"); Serial.print(wy); Serial.print("\t");
  // Serial.print("wz:"); Serial.print(wz); Serial.print("\t");

  // 1. 
  float rg=r_kf+dt*(wx+wy*sin(r_kf)*tan(p_kf)+wz*cos(r_kf)*tan(p_kf));
  float pg=p_kf+dt*(wy*cos(r_kf)-wz*sin(r_kf));
  // Serial.print("rg: "); Serial.print(rg); Serial.print("\t");
  // Serial.print("pg: "); Serial.print(rg); Serial.print("\t");

  // 2. 
  float A11=(wy*cos(r_kf)-wz*sin(r_kf))*tan(p_kf);
  float A12=(wy*sin(r_kf)+wz*cos(r_kf))/(cos(p_kf))/(cos(p_kf));
  float A21=-wy*sin(r_kf)-wz*cos(r_kf);
  float A22=0;
  // P[0][0]=P[0][0]+dt*(P[0][0]*A11+P[1][0]*A12+P[0][0]*A11+P[0][1]*A12+Q[0][0]);
  // P[0][1]=P[0][1]+dt*(P[0][1]*A11+P[1][1]*A12+P[0][0]*A21+P[0][1]*A22+Q[0][1]);
  // P[1][0]=P[1][0]+dt*(P[0][0]*A21+P[1][0]*A22+P[1][0]*A11+P[1][1]*A12+Q[1][0]);
  // P[1][1]=P[1][1]+dt*(P[0][1]*A21+P[1][1]*A22+P[1][0]*A21+P[1][1]*A22+Q[1][1]);

  P[0][0]=dt*((P[0][0]*A11+P[1][0]*A12)*A11 + (P[0][1]*A11+P[1][1]*A12)*A12 + Q[0][0]);
  P[0][1]=dt*((P[0][0]*A11+P[1][0]*A12)*A21 + (P[0][1]*A11+P[1][1]*A12)*A22 + Q[0][1]);
  P[1][0]=dt*((P[0][0]*A21+P[1][0]*A22)*A11 + (P[0][1]*A21+P[1][1]*A22)*A12 + Q[1][0]);
  P[1][1]=dt*((P[0][0]*A21+P[1][0]*A22)*A21 + (P[0][1]*A21+P[1][1]*A22)*A22 + Q[1][1]);

  // Serial.print("A11:"); Serial.print(A11); Serial.print("\t");
  // Serial.print("A12:"); Serial.print(A12); Serial.print("\t");
  // Serial.print("A21:"); Serial.print(A21); Serial.print("\t");
  // Serial.print("A22:"); Serial.print(A22); Serial.print("\t");

  // Serial.print("P11:"); Serial.print(P[0][0]); Serial.print("\t");
  // Serial.print("P12:"); Serial.print(P[1][0]); Serial.print("\t");
  // Serial.print("P21:"); Serial.print(P[0][1]); Serial.print("\t");
  // Serial.print("P22:"); Serial.print(P[1][1]); Serial.print("\t");


  // 3. 
  float C11=0;
  float C12=g*cos(p_kf);
  float C21=-g*cos(r_kf)*cos(p_kf);
  float C22=g*sin(r_kf)*sin(p_kf);
  float C31=g*sin(r_kf)*cos(p_kf);
  float C32=g*sin(p_kf)*cos(r_kf);

  // Serial.print("C11:"); Serial.print(C11); Serial.print("\t");
  // Serial.print("C21:"); Serial.print(C21); Serial.print("\t");
  // Serial.print("C31:"); Serial.print(C31); Serial.print("\t");
  // Serial.print("C12:"); Serial.print(C12); Serial.print("\t");
  // Serial.print("C22:"); Serial.print(C22); Serial.print("\t");
  // Serial.print("C32:"); Serial.print(C32); Serial.print("\t");

  float a=P[0][0]*C11+P[0][1]*C12;
  float b=P[0][0]*C21+P[0][1]*C22;
  float c=P[0][0]*C31+P[0][1]*C32;
  float d=P[1][0]*C11+P[1][1]*C12;
  float e=P[1][0]*C21+P[1][1]*C22;
  float f=P[1][0]*C31+P[1][1]*C32;

  float D11=C11*(P[0][0]*C11+P[1][0]*C12)+C12*(P[0][1]*C11+P[1][1]*C12)+R[0][0];
  float D21=C11*(P[0][0]*C21+P[1][0]*C22)+C12*(P[0][1]*C21+P[1][1]*C22);
  float D31=C11*(P[0][0]*C31+P[1][0]*C32)+C12*(P[0][1]*C31+P[1][1]*C32);

  // Serial.print("D11:"); Serial.print(D11); Serial.print("\t");
  // Serial.print("D21:"); Serial.print(D21); Serial.print("\t");
  // Serial.print("D31:"); Serial.print(D31); Serial.print("\t");

  float D12=C21*(P[0][0]*C11+P[1][0]*C12)+C22*(P[0][1]*C11+P[1][1]*C12);
  float D22=C21*(P[0][0]*C21+P[1][0]*C22)+C22*(P[0][1]*C21+P[1][1]*C22)+R[1][1];
  float D32=C21*(P[0][0]*C31+P[1][0]*C32)+C22*(P[0][1]*C31+P[1][1]*C32);

  // Serial.print("D12:"); Serial.print(D12); Serial.print("\t");
  // Serial.print("D22:"); Serial.print(D22); Serial.print("\t");
  // Serial.print("D32:"); Serial.print(D32); Serial.print("\t");

  float D13=C31*(P[0][0]*C11+P[1][0]*C12)+C32*(P[0][1]*C11+P[1][1]*C12);
  float D23=C31*(P[0][0]*C21+P[1][0]*C22)+C32*(P[0][1]*C21+P[1][1]*C22);
  float D33=C31*(P[0][0]*C31+P[1][0]*C32)+C32*(P[0][1]*C31+P[1][1]*C32)+R[2][2];

  // Serial.print("D13:"); Serial.print(D13); Serial.print("\t");
  // Serial.print("D23:"); Serial.print(D23); Serial.print("\t");
  // Serial.print("D33:"); Serial.print(D33); Serial.print("\t");

  float den=D21*D13*D32-D13*D22*D31+D23*D12*D31+D33*(-D21*D12+D11*D22)-D23*D11*D32;

  K[0][0]=(a*(-D23*D32+D22*D33)+b*(D23*D31-D21*D33)+c*(-D22*D31+D21*D32))/den;
  K[1][0]=(d*(-D23*D32+D22*D33)+e*(D23*D31-D21*D33)+f*(-D22*D31+D21*D32))/den;

  K[0][1]=(-a*(D12*D33-D13*D32)+b*(D11*D33-D13*D31)-c*(D11*D32-D12*D31))/den;
  K[1][1]=(-d*(D12*D33-D13*D32)+e*(D11*D33-D13*D31)-f*(D11*D32-D12*D31))/den;

  K[0][2]=(-a*(D13*D22-D23*D12)-b*(D23*D11-D21*D13)+c*(D11*D22-D21*D12))/den;
  K[1][2]=(-d*(D13*D22-D23*D12)-e*(D23*D11-D21*D13)+f*(D11*D22-D21*D12))/den;

  // Serial.print("K11:"); Serial.print(K[0][0]); Serial.print("\t");
  // Serial.print("K12:"); Serial.print(K[1][0]); Serial.print("\t");
  // Serial.print("K21:"); Serial.print(K[0][1]); Serial.print("\t");
  // Serial.print("K22:"); Serial.print(K[1][1]); Serial.print("\t");
  // Serial.print("DEN:"); Serial.print(den); Serial.print("\t");
  
  // https://www.symbolab.com/solver/matrix-calculator/%5Cbegin%7Bpmatrix%7Da%26b%26c%5C%5C%20%20%20d%26e%26f%5Cend%7Bpmatrix%7D%5Cbegin%7Bpmatrix%7DX%26Y%26Z%5C%5C%20%20%20L%26M%26N%5C%5C%20%20%20O%26P%26Q%5Cend%7Bpmatrix%7D%5E%7B-1%7D?or=input

  // 4.
  float d1=ax_float-g*sin(p_kf);
  float d2=ay_float+g*cos(p_kf)*sin(r_kf);
  float d3=-az_float+g*cos(p_kf)*cos(r_kf); // add a negative sign since roll and pitch are swapped

  r_kf=rg+K[0][0]*d1+K[0][1]*d2+K[0][2]*d3;
  p_kf=pg+K[1][0]*d1+K[1][1]*d2+K[1][2]*d3;

  // 5. 
  float p1=1-K[0][0]*C11+K[0][1]*C21+K[0][2]*C31;
  float p2=K[0][0]*C12+K[0][1]*C22+K[0][2]*C32;
  float p3=K[1][0]*C11+K[1][1]*C21+K[1][2]*C31;
  float p4=1-K[1][0]*C12+K[1][1]*C22+K[1][2]*C32;

  P[0][0]=p1*P[0][0]+p2*P[1][0];
  P[0][1]=p1*P[0][1]+p2*P[1][1];
  P[1][0]=p3*P[0][0]+p4*P[1][0];
  P[1][1]=p3*P[0][1]+p4*P[1][1];

  // float pitch_norm=-(p_kf*180/M_PI+180 % 360)-180;
  // float roll_norm=(r_kf*180/M_PI+180 % 360)-180;

  float pitch_norm=_normalize_angle(p_kf);
  float roll_norm=_normalize_angle(r_kf);

  // if (pitch_norm>(M_PI))  pitch_norm-=2*M_PI;
  // Serial.print("P1:"); Serial.print(P[0][0]); Serial.print("\t");
  // Serial.print("P2:"); Serial.print(P[0][1]); Serial.print("\t");
  // Serial.print("P3:"); Serial.print(P[1][0]); Serial.print("\t");
  // Serial.print("P4:"); Serial.print(P[1][1]); Serial.print("\t");
  // Serial.print("p:"); Serial.print(pitch*180/M_PI); Serial.print("\t"); // Serial.print("\t");
  // Serial.print("r:"); Serial.print(roll*180/M_PI); Serial.print("\t");
  // Serial.print("y:"); Serial.print(yaw*180/M_PI); Serial.print("\t");
  // Serial.print("p_kf:"); 
  Serial.print(-roll_norm*180/M_PI); 
  Serial.print(" ");
  // Serial.print("\t"); // Serial.print("\t");
  // Serial.print("r_kf:"); 
  Serial.print(-pitch_norm*180/M_PI); 
  Serial.print(" ");
  // Serial.print("\t");
  Serial.print(180); // To freeze the lower limit
  Serial.print(" ");

  Serial.print(-180); // To freeze the upper limit
  Serial.print(" ");
  // Serial.print("yaw:"); Serial.print(yaw*180/M_PI); Serial.print("\t");
  // Serial.print("yaw_v:"); Serial.print(yaw_v*180/M_PI); Serial.print("\t");
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////

// float _normalize_angle(float angle){
//   float temp=fmod(angle, 2*M_PI);
//   return temp>= 0 ? temp : (temp+2*M_PI);
// }

float _normalize_angle(float angle) {
    float temp = fmod(angle + M_PI, 2 * M_PI);
    return temp >= 0 ? (temp - M_PI) : (temp + M_PI);
}