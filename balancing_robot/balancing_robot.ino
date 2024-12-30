#include <MsTimer2.h>        //internal timer 2
#include <PinChangeInt.h>    //this library can make all pins of arduino UNO as external interrupt
#include <MPU6050.h>      //MPU6050 library 
#include <Wire.h>        //IIC communication library 

MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     //Instantiate an MPU6050 object; name mpu6050

/*Pins*/
#define VOLTAGE_PIN A2
#define ECHO_PIN A3
#define TRIG_PIN 11
#define AIN1 7
#define AIN2 3
#define PWMA_LEFT 5
#define BIN1 13
#define BIN2 12
#define PWMB_RIGHT 6
#define STANDBY_PIN 8
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4
#define LEFT_RECEIVE_PIN A0
#define RIGHT_RECEIVE_PIN A1

///////////////////////angle parameters//////////////////////////////
float Angle;
float angle_X; //calculate the inclined angle variable of X-axis by accelerometer
float angle_Y; //calculate the inclined angle variable of Y-axis by accelerometer
float angle0 = 0; //Actual measured angle (ideally 0 degrees) 
float Gyro_x,Gyro_y,Gyro_z;  //Angular angular velocity for gyroscope calculation
///////////////////////angle parameters//////////////////////////////

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001;  //Covariance of gyroscope noise
float Q_gyro = 0.004;    //Covariance of gyroscope drift noise
float R_angle = 0.5;    //Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time.
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    //gyroscope drift

float accelz = 0;
float angle;
float angleY_one;
float angle_speed;

float angle_yaw, angle_pitch, angle_roll;

float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////

//////////////////////PD parameters///////////////////////////////
double kp = 55, ki = 1.6, kd = 0.95;                   //Angle loop parameter kp = 55, ki = 1.6, kd = 0.95
double previous_error = 0, error = 0;
double pid_i = 0;
double setp0 = 0; //Angle balance point
int balance_pid, speed_control_pid;  //angle output
float right_pwm=0,left_pwm=0, self_balance_pid_setpoint = 0;
int maxPwm = 255;

// Parameters for counting motors speed
volatile uint32_t encoder_count_right_a = 0, encoder_count_left_a = 0;
uint8_t speed_control_period_count;
int encoder_left_pulse_num_speed = 0, encoder_right_pulse_num_speed = 0;
double speed_filter = 0.0, car_speed_integeral = 0.0, speed_filter_old = 0.0;
int motor_pid = 0;
int setting_car_speed = 0, turn_speed = 0;
int rotation_pid = 0;

// PID Paramters 
float kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Ultrasonic Parameters
uint8_t measure_flag = 2;
uint32_t measure_prev_time = 0;
double distance_value = 0.0;
uint32_t get_distance_prev_time = 0;

void setup() 
{
  //set the control motor’s pin to OUTPUT
  pinMode(BIN1,OUTPUT);       
  pinMode(BIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(STANDBY_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PWMB_RIGHT,OUTPUT);
  pinMode(PWMA_LEFT,OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);

  //Initial state value
  digitalWrite(BIN1,1);
  digitalWrite(BIN2,0);
  digitalWrite(AIN1,0);
  digitalWrite(AIN2,1);
  analogWrite(PWMB_RIGHT,0);
  analogWrite(PWMA_LEFT,0);
  digitalWrite(STANDBY_PIN, HIGH);

  // Join I2C bus
  Wire.begin();                            //Join the I2C bus sequence
  Serial.begin(9600);                       //open serial monitor, set the baud rate to 9600
  delay(1500);
  mpu6050.initialize();                       //initialize MPU6050
  delay(2);

  //5ms  use timer2 to set the timer interrupt (Note: using timer2 will affect the PWM output of pin3 pin11.)
  MsTimer2::set(5, balance_robot);    // 5ms execute the function DSzhongduan once
  MsTimer2::start();    // start the interrupt
}

void loop() 
{ 
}

/////////////////////////////////interrupt////////////////////////////
void balance_robot()
{
  sei();  //Allow overall interrupt
  countpulse();
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);      //get angle and Kalman_Filter
  PID();         // angle loop of PD control
  anglePWM();
}
///////////////////////////////////////////////////////////


/////////////////////////////angle calculation///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  Angle = -atan2(ay , az) * (180/ PI);           //Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //  Kalman Filter
  //Rotation Angle Z axis parameter
  Gyro_z = -gz / 131;                      //Z-axis angular velocity
  //accelz = az / 16.4;

  float angleAx = -atan2(ax, az) * (180 / PI); //Calculate the angle with the x-axis
  Gyro_y = -gy / 131.00; //Y-axis angular velocity
  Yiorderfilter(angleAx, Gyro_y); //first-order filter
}
////////////////////////////////////////////////////////////////

///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m){
  angle += (gyro_m - q_bias) * dt;          //Prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //Differential of azimuth error covariance
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //A prior estimation error covariance differential integral
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //Denominator 
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  //Intermediate variable of matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    //Posterior estimate
  angle_speed = gyro_m - q_bias;   //The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; ////Posterior estimation to get the optimal angle
}

/////////////////////first-order Filter/////////////////
void Yiorderfilter(float angle_m, float gyro_m){
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

//////////////////angle PD////////////////////
void PID()
{
  error = angle - angle0 - self_balance_pid_setpoint;
  pid_i += error*ki;
  pid_i = constrain(pid_i, -255, 255);
  balance_pid = kp * (angle + angle0 - self_balance_pid_setpoint) + pid_i + kd * angle_speed; //PID angle loop control
  if(angle0 == 0){
    if(balance_pid < 0)self_balance_pid_setpoint += 0.00008;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards value = 0.0015
    if(balance_pid > 0)self_balance_pid_setpoint -= 0.00008;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards value = 0.0015
  }
}

void motor_pwm(void){
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    motor_pid = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_pid = turn_speed + kd_turn*Gyro_z;
  }
}


////////////////////////////PWM end value/////////////////////////////
void anglePWM()
{
  
  left_pwm = -balance_pid;
  right_pwm =-balance_pid;
  
  if(right_pwm>maxPwm)             //limit PWM value not greater than 255
  {
    right_pwm = maxPwm;
  }
  else if(right_pwm<-maxPwm) 
  {
    right_pwm=-maxPwm;
  }
  if(left_pwm>maxPwm)
  {
    left_pwm=maxPwm;
  }
  else if(left_pwm<-maxPwm)
  {
    left_pwm=-maxPwm;
  }
  if(angle>30 || angle<-30)      //When the self-balancing trolley’s tilt angle is greater than 30 degrees, the motor will stop.
  {
    right_pwm=left_pwm=0;
  }

  if(left_pwm>=0)         //determine the motor’s steering and speed by the positive and negative of PWM 
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2,LOW);
    analogWrite(PWMA_LEFT,left_pwm);
  }
  else
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    analogWrite(PWMA_LEFT,-left_pwm);
  }

  if(right_pwm>=0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB_RIGHT,right_pwm);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB_RIGHT,-right_pwm);
  }
}

