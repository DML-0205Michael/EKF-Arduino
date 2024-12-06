void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void Qua_update(float gx，float gy，float gz，float ax，float ay，float az)
{
static float exInt＝0，eyInt＝0，ezInt＝0；//定义积分，我们融合中一 般不使用积分，只有当角度收敛慢时可以适当给一点积分加速收敛。
static float Kp＝7.2f，Ki＝0.00f，halfT＝0.0025f；//设置采样周期为 5ms，则法f/2为0.0025ms
float tmp_q0，tmp_q1，tmp_q2，tmp_q3；//计算四元数缓冲
float size；//归一化模
float vx，vy，vz；//四元数估算重力分量大小
float ex，ey，ez；
float q0q0＝q0*q0；
float q0q0＝q0*q1；
float q0q0＝q0*q2；
float q0q0＝q0*q3；
float q0q0＝q0*q1；
float q0q0＝q0*q2；
float q0q0＝q0*q3；
float q0q0＝q0*q2；
float q0q0＝q0*q3；
float q3q3＝q3*q3；//预计算，用以减少重复计算
size＝sqrt(ax*ax+ay*ay+az*az)；//归一化加速度计模
ax＝ax/size；
ay＝ay/size；
az＝az/size；//加速度计算重力分量大小
vx＝2*(q1q3-q0q2)；
vy＝2*(q0q1+q2q3)；
vz＝q0q0-q1q1-q2q2+q3q3；//分解姿态四元数中的重力 分量
ex＝(ay*vz-az*vy)；
ey＝(az*vx-ax*vz)；
ez＝(ax*vy-ay*vx)；//计算角速度级测量的角度和姿态四元 数的角度
exInt＝exInt+ex*Ki；
eyInt＝eyInt+ey*Ki；
ezInt＝ezInt+ez*Ki；
/*
if(fabs(ex)＜＝0.01)
exInt＝0；
if(fabs(ey)＜＝0.01)
eyInt＝0；
if(fabs(ez)＜＝0.01)
ezInt＝0；*///在此不使用积分
gx＝gx+Kp*ex+exInt；
gy＝gy+Kp*ey+eyInt；
gz＝gz+Kp*ez+ezInt；//互补滤波
tmp_q0＝q0+(-q1*gx-q2*gy-q3*gz)*halfT；
tmp_q1＝q1+(q0*gx+q2*gz-q3*gy)*halfT；
tmp_q2＝q2+(q0*gy-q1*gz+q3*gx)*halfT；
tmp_q3＝q3+(q0*gz+q1*gy-q2*gx)*halfT；//一定要使用 缓冲
q0＝tmp_q0；
q1＝tmp_q1；
q2＝tmp_q2；
q3＝tmp_q3；//更新四元数
size＝sqrt(q0*q0+q1*q1+q2*q2+q3*q3)；//归一化姿态 四元数模
q0＝q0/size；
q1＝q1/size；
q2＝q2/size；
q3＝q3/size；//规范化姿态四元数
pitch＝asinf(-2*q1*q3+2*q0*q2)*57.296f；//pitch
(57.296为换算单位180/π)
roll＝atan2f(2*q2*q3+2*q0*q1，-2*q1*q1-2*q2* q2+1)*57.296f；
//roll其中atan2f函数比atan稳定，不会出现90度的反正 切奇异值。
yaw＝atan2f(2*(q1*q2+q0*q3)，q0*q0+q1*q1-q2*q2-q3*q3)* 57.296f；
//yaw四旋翼中不使用角度控制yaw
}//5ms执行一次四元数姿态解算函数。