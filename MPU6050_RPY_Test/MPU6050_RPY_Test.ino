////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu;

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s
Kalman kalmanY; // Create the Kalman instances
Kalman kalmanX;
double p_kf=0,r_kf=0;
uint32_t timer_kf;
float yaw_v=0, yaw=0;
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

  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float))); 
  double roll=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
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
  
  Qua_update(wx, wy, wz, ax_float, ay_float, az_float);

  // double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
  // timer_kf = micros();
  
  // double pitch=atan2f(ay_float,az_float);
  double pitch=atan2f(ay_float,sqrt(sq(ax_float)+sq(az_float))); // *180/M_PI; 
  double roll=atan2f(-1*ax_float,sqrt(sq(ay_float)+sq(az_float))); // *180/M_PI;
  double yaw=atan(sqrt(sq(ax_float)+sq(ay_float))/az_float);
  // r_kf = kalmanX.getAngle(roll, wy, dt); // roll, rad
  // p_kf = kalmanY.getAngle(pitch, wx, dt); // pitch, rad

  

  // yaw_v = (wz - wx * sin(p_kf) + wy * cos(p_kf) * sin(r_kf)) / (cos(r_kf) * cos(p_kf));

  // yaw=yaw+yaw_v*0.005;
  // yaw=fmod(yaw,2*M_PI);
  // yaw>=0 ? yaw : (yaw+2*M_PI);

  // // float temp=r_kf;
  // // r_kf=-p_kf;
  // // p_kf=temp;
  
  Serial.print("p:"); Serial.print(pitch*180/M_PI); Serial.print("\t"); // Serial.print("\t");
  Serial.print("r:"); Serial.print(roll*180/M_PI); Serial.print("\t");
  Serial.print("y:"); Serial.print(yaw*180/M_PI); Serial.print("\t");
  // Serial.print("p_kf:"); Serial.print(p_kf*180/M_PI); Serial.print("\t"); // Serial.print("\t");
  // Serial.print("r_kf:"); Serial.print(r_kf*180/M_PI); Serial.print("\t");
  // Serial.print("yaw:"); Serial.print(yaw*180/M_PI); Serial.print("\t");
  // Serial.print("yaw_v:"); Serial.print(yaw_v*180/M_PI); Serial.print("\t");
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////
float q0=1;
float q1=0;
float q2=0;
float q3=0;

void Qua_update(float gx,float gy,float gz,float ax,float ay,float az){
  // https://patents.google.com/patent/CN109931929A/zh
static float exInt=0,eyInt=0,ezInt=0;//定义积分,我们融合中一 般不使用积分,只有当角度收敛慢时可以适当给一点积分加速收敛。
static float Kp=7.2f,Ki=0.00f,halfT=0.0025f;//设置采样周期为 5ms,则法f/2为0.0025ms
float tmp_q0,tmp_q1,tmp_q2,tmp_q3;//计算四元数缓冲
float size;//归一化模
float vx,vy,vz;//四元数估算重力分量大小
float ex,ey,ez;
// float q0q0=q0*q0;
// float q0q1=q0*q1;
// float q0q2=q0*q2;
// float q0q3=q0*q3;
// float q0q0=q0*q1;
// float q0q0=q0*q2;
// float q0q0=q0*q3;
// float q0q0=q0*q2;
// float q0q0=q0*q3;
// float q3q3=q3*q3;//预计算,用以减少重复计算
size=sqrt(ax*ax+ay*ay+az*az);//归一化加速度计模
ax=ax/size;
ay=ay/size;
az=az/size;//加速度计算重力分量大小
vx=2*(q1*q3-q0*q2);
vy=2*(q0*q1+q2*q3);
vz=q0*q0-q1*q1-q2*q2+q3*q3;//分解姿态四元数中的重力 分量
ex=(ay*vz-az*vy);
ey=(az*vx-ax*vz);
ez=(ax*vy-ay*vx);//计算角速度级测量的角度和姿态四元 数的角度
exInt=exInt+ex*Ki;
eyInt=eyInt+ey*Ki;
ezInt=ezInt+ez*Ki;
/*
if(fabs(ex)＜=0.01)
exInt=0;
if(fabs(ey)＜=0.01)
eyInt=0;
if(fabs(ez)＜=0.01)
ezInt=0;*///在此不使用积分
gx=gx+Kp*ex+exInt;
gy=gy+Kp*ey+eyInt;
gz=gz+Kp*ez+ezInt;//互补滤波
tmp_q0=q0+(-q1*gx-q2*gy-q3*gz)*halfT;
tmp_q1=q1+(q0*gx+q2*gz-q3*gy)*halfT;
tmp_q2=q2+(q0*gy-q1*gz+q3*gx)*halfT;
tmp_q3=q3+(q0*gz+q1*gy-q2*gx)*halfT;//一定要使用 缓冲
q0=tmp_q0;
q1=tmp_q1;
q2=tmp_q2;
q3=tmp_q3;//更新四元数
size=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);//归一化姿态 四元数模
q0=q0/size;
q1=q1/size;
q2=q2/size;
q3=q3/size;//规范化姿态四元数
float roll=asinf(-2*q1*q3+2*q0*q2)*57.296f;//pitch (57.296为换算单位180/π)
float pitch=atan2f(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.296f;
//roll其中atan2f函数比atan稳定,不会出现90度的反正 切奇异值。
yaw=atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)* 57.296f;
//yaw四旋翼中不使用角度控制yaw
  Serial.print("p_kf:"); Serial.print(pitch); Serial.print("\t"); // Serial.print("\t");
  Serial.print("r_kf:"); Serial.print(roll); Serial.print("\t");
  Serial.print("yaw:"); Serial.print(yaw); Serial.print("\t");
}//5ms执行一次四元数姿态解算函数。