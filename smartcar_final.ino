#include <NewPing.h>   
#include <TimerOne.h>  // 타이머 라이브러리 추가  
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#define middle_TRIGGER_PIN  2  
#define middle_ECHO_PIN     4  
#define left_TRIGGER_PIN  9  
#define left_middle_ECHO_PIN  10
#define right_TRIGGER_PIN  6
#define right_middle_ECHO_PIN  13
#define MOTOR_A_DIRECTION1_PIN 11
#define MOTOR_A_DIRECTION2_PIN 12
#define MOTOR_A_SPEED_PIN 3
#define MOTOR_B_DIRECTION1_PIN 7
#define MOTOR_B_DIRECTION2_PIN 8
#define MOTOR_B_SPEED_PIN 5
#define MAX_DISTANCE 200 
MPU9250 accelgyro;
I2Cdev   I2C_M;
volatile int16_t mx, my, mz;
volatile int16_t ax, ay, az;
volatile int16_t gx, gy, gz;
volatile uint8_t buffer_m[6];
volatile float Mxyz[3];
#define sample_num_mdate  5000
volatile float heading;
volatile float tiltheading;

volatile float Axyz[3];
volatile float Gxyz[3];

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

//static float mx_centre = 0.4;
//static float my_centre = 131;
//static float mz_centre = -50;
        //82.00         -40.00

static float mx_centre = 28.00 ;
static float my_centre = 82.00;
static float mz_centre = -40.00;
volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;
void getAccel_Data(void);
void getCompassDate_calibrated ();
void get_one_sample_date_mxyz();
void getCompass_Data(void);
void Mxyz_init_calibrated ();
void getHeading(void);
void getTiltHeading(void);
volatile int flag_ir1=0;
volatile int flag_ir2=0;
volatile int flag_ir3=0;
volatile int flag_dir=0;
volatile int flag_cnt=0;
NewPing middle_sonar(middle_TRIGGER_PIN, middle_ECHO_PIN, MAX_DISTANCE); 
NewPing left_sonar(left_TRIGGER_PIN, left_middle_ECHO_PIN, MAX_DISTANCE); 
NewPing right_sonar(right_TRIGGER_PIN, right_middle_ECHO_PIN, MAX_DISTANCE); 
volatile float init_heading[5];
const int FILTER_SIZE = 3;
void move_forward(  int delay_par = 0, int init_A_Speed = 80,   int init_B_Speed = 90);
void move_backward( int delay_par = 0, int init_A_Speed = 80,   int init_B_Speed = 90);
void move_stop(     int delay_par = 0, int init_A_Speed = 0,    int init_B_Speed = 0);
void move_right(    int delay_par = 0, int init_A_Speed = 120,  int init_B_Speed = 80);
void move_left(     int delay_par = 0, int init_A_Speed = 80,   int init_B_Speed = 120);
int medianFilter( int* values, int size) {
  int sortedValues[FILTER_SIZE];
  for (int i = 0; i < size; i++) {
    sortedValues[i] = values[i];
  }
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (sortedValues[j] > sortedValues[j + 1]) {
        int temp = sortedValues[j];
        sortedValues[j] = sortedValues[j + 1];
        sortedValues[j + 1] = temp;
      }
    }
  }
  return sortedValues[size / 2];
}
float medianFilter_float( float* values, int size) {
  int sortedValues[FILTER_SIZE];
  for (int i = 0; i < size; i++) {
    sortedValues[i] = values[i];
  }
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (sortedValues[j] > sortedValues[j + 1]) {
        float temp = sortedValues[j];
        sortedValues[j] = sortedValues[j + 1];
        sortedValues[j + 1] = temp;
      }
    }
  }
  return sortedValues[size / 2];
}
void setup() {
  Serial.begin(9600); 
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
 
  delay(1000);
  Serial.println("     ");
  pinMode(MOTOR_A_DIRECTION1_PIN, OUTPUT);
  pinMode(MOTOR_A_DIRECTION2_PIN, OUTPUT); 
  pinMode(MOTOR_B_DIRECTION1_PIN, OUTPUT);
  pinMode(MOTOR_B_DIRECTION2_PIN, OUTPUT);       
  // Mxyz_init_calibrated ();
  getAccel_Data();

  delay(500);
  for (int i = 0; i < FILTER_SIZE; i++) {
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();    
    init_heading[i] = heading;
    
  }
  // 메디안 필터를 사용하여 초기 heading값 구하기
  init_heading[0] = medianFilter_float(init_heading, FILTER_SIZE);
  //init_heading[0] = init_heading[0]+10;
  Serial.print("init: "); 
  Serial.println(init_heading[0]); 
  delay(500);
  Timer1.initialize(100000);  // 100ms 
  Timer1.attachInterrupt(mStimer10);
    
}
void loop() {

  //int filteredDistance1 = left_sonar.ping_cm();
  //int filteredDistance2 = middle_sonar.ping_cm();
  //int filteredDistance3 = right_sonar.ping_cm();  
  // 초음파 센서로부터 거리 값을 측정
  int distances1[FILTER_SIZE];
  int distances2[FILTER_SIZE];
  int distances3[FILTER_SIZE];
  for (int i = 0; i < FILTER_SIZE; i++) {
    distances1[i] = left_sonar.ping_cm();
    distances2[i] = middle_sonar.ping_cm();
    distances3[i] = right_sonar.ping_cm();
  }
  // 메디안 필터를 사용하여 거리 값을 추정    
  int filteredDistance1 = medianFilter(distances1, FILTER_SIZE);
  int filteredDistance2 = medianFilter(distances2, FILTER_SIZE);
  int filteredDistance3 = medianFilter(distances3, FILTER_SIZE);


 
  Serial.print("dist: ");
  Serial.print(filteredDistance1);
  Serial.print(",");
  Serial.print(filteredDistance2);
  Serial.print(",");
  Serial.print(filteredDistance3);    
  Serial.print("    ");              
  Serial.print("Compass Value of X,Y,Z, heading : ");
  Serial.print(Mxyz[0]);
  Serial.print(",");
  Serial.print(Mxyz[1]);
  Serial.print(",");
  Serial.print(Mxyz[2]);
  Serial.print(",");
  Serial.print(heading);   
  Serial.print(",");
  Serial.print(init_heading[0]); 
  Serial.print(", flagir: ");
  Serial.print(flag_ir1); 
  Serial.print(",");
  Serial.print(flag_ir2); 
  Serial.print(",");
  Serial.print(flag_ir3); 
  Serial.print(",");
  Serial.print(flag_dir); 
  Serial.print(",");
  if (flag_dir == 1){
      move_stop(100);
      move_backward(1000);     
      Serial.print(",   ");
      Serial.println("move_backward irsensor1");                
  }
  else if (flag_dir == 2){
      move_stop(100);
      move_backward(1000);  
      Serial.print(",   ");
      Serial.println("move_backward irsensor2");      
  }
  else if (flag_dir ==3){
      move_stop(100);
      move_backward(1000);  
    Serial.print(",   ");
    Serial.println("move_backward irsensor3");             
  }
  else if ( filteredDistance1<10 && filteredDistance1>2){
    if ( (filteredDistance2 < 5 && filteredDistance2 > 1)){
      move_stop(100);
      move_backward(1000);    
      move_stop(100);
      move_right(200);   
    }
    else{
      move_stop(100);
      move_backward(800);    
      move_stop(100);
      move_right(200);    
    }    
    Serial.print(",   ");
    Serial.println("move_right ultrawave");  
  }
  else if (filteredDistance2<15 && filteredDistance2>2){
    move_stop(100);
    move_backward(800);    
    move_stop(100);  
    if (filteredDistance1 > filteredDistance3) {
      move_left(400, 70, 100);
        Serial.print(",   ");
        Serial.println("move_left ultrawave"); 
    }
    else if (filteredDistance1 < filteredDistance3){
      move_right(200);
      Serial.print(",   ");
      Serial.println("move_right ultrawave");  
    }  
  }
  else if (filteredDistance3<10 && filteredDistance3>2){
    if ( filteredDistance2 < 5 && filteredDistance2 > 1){
        move_stop(100);
        move_backward(1000);    
        move_stop(100);
        move_left(180);    
    }  
    else{
      move_stop(100);
      move_backward(800);    
      move_stop(100);
      move_left(180);     
    }   
    Serial.print(",   ");
    Serial.println("move_left ultrawave");  
  }
  
  else {
    //move_forward();
    //Serial.print(",   ");
    //Serial.println("move_forward");   
    getGyro_Data();
    getCompassDate_calibrated(); 
    getHeading();    

    if (  (init_heading[0]-6)<=heading && (init_heading[0]+6)>=heading  ){
      move_forward(200, 90, 110);   
      Serial.print(",   ");
      Serial.println("move_forward");  
    }      
    else if( init_heading[0]-6 > heading ){

      move_left(0,0,100);
      Serial.print(",   ");
      Serial.println("move_left");    

    }
    else if (init_heading[0]+6 < heading) {
      move_right(0,90,0);
      Serial.print(",   ");
      Serial.println("move_right"); 
    }             
  }  

}
void mStimer10() {
  flag_cnt ++;
  if (flag_cnt>=10){
    flag_cnt = 0;    
  }
  if ( analogRead(A0) < 800 ){   //800미만은 빛반사 감자한것(수정가능). 검정색 감지 =  허공 감지
    flag_ir1 ++;    
  }
  else {
    flag_ir1 -= 3;
    if (flag_ir1 < 0) flag_ir1 = 0;
  }
  if ( analogRead(A1) < 800 ){
    flag_ir2 ++;
  }
  else {
    flag_ir2 -= 3;
    if (flag_ir2 < 0) flag_ir2 = 0;
  }

  if ( analogRead(A2) < 800 ){
    flag_ir3 ++;
  }
  else {
    flag_ir3 -= 3;
    if (flag_ir3 < 0) flag_ir3 = 0;
  }

  if (flag_ir1 >= 20){
    flag_dir = 1; flag_ir1 = 20;
  }
  else if(flag_ir2>= 20){
    flag_dir = 2; flag_ir2 = 20;
  }
  else if(flag_ir3>=20){
    flag_dir = 3; flag_ir3 = 20;
  }
  else{
    flag_dir = 0;
  }
  
}

void move_forward(int delay_par = 0, int init_A_Speed = 80, int init_B_Speed = 90){
  digitalWrite(MOTOR_A_DIRECTION1_PIN, HIGH); 
  digitalWrite(MOTOR_A_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_A_SPEED_PIN, init_A_Speed);   
  digitalWrite(MOTOR_B_DIRECTION1_PIN, HIGH); 
  digitalWrite(MOTOR_B_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_B_SPEED_PIN, init_B_Speed);   
  delay(delay_par);
}
void move_backward(int delay_par = 0, int init_A_Speed = 80, int init_B_Speed = 90){
  digitalWrite(MOTOR_A_DIRECTION1_PIN, LOW); 
  digitalWrite(MOTOR_A_DIRECTION2_PIN, HIGH); 
  analogWrite(MOTOR_A_SPEED_PIN, init_A_Speed);    
  digitalWrite(MOTOR_B_DIRECTION2_PIN, LOW); 
  digitalWrite(MOTOR_B_DIRECTION2_PIN, HIGH); 
  analogWrite(MOTOR_B_SPEED_PIN, init_B_Speed); 
  delay(delay_par);
}
void move_stop(int delay_par = 0, int init_A_Speed = 0, int init_B_Speed = 0){
  digitalWrite(MOTOR_A_DIRECTION1_PIN, LOW);  
  digitalWrite(MOTOR_A_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_A_SPEED_PIN, init_A_Speed);    
  digitalWrite(MOTOR_B_DIRECTION1_PIN, LOW);  
  digitalWrite(MOTOR_B_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_B_SPEED_PIN, init_B_Speed); 
  delay(delay_par); 
}
void move_right(int delay_par = 0, int init_A_Speed = 120, int init_B_Speed = 80){
  digitalWrite(MOTOR_A_DIRECTION1_PIN, HIGH);  
  digitalWrite(MOTOR_A_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_A_SPEED_PIN, init_A_Speed);    
  digitalWrite(MOTOR_B_DIRECTION1_PIN, HIGH);  
  digitalWrite(MOTOR_B_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_B_SPEED_PIN, init_B_Speed); 
  delay(delay_par);
}
void move_left(int delay_par = 0, int init_A_Speed = 80, int init_B_Speed = 120){
  digitalWrite(MOTOR_A_DIRECTION1_PIN, HIGH);  
  digitalWrite(MOTOR_A_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_A_SPEED_PIN, init_A_Speed);    
  digitalWrite(MOTOR_B_DIRECTION1_PIN, HIGH);  
  digitalWrite(MOTOR_B_DIRECTION2_PIN, LOW); 
  analogWrite(MOTOR_B_SPEED_PIN, init_B_Speed); 
  delay(delay_par);
}
void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  //if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated ()
{

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while (!Serial.find("ready"));
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}


void get_calibration_Data ()
{
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();

    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];

  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;
}

void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}
