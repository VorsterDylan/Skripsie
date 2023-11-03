//Library use
#include <math.h>
#include <PID_v1.h>
#include <LittleFS.h>
#include <Arduino.h>

//pin defintions  
//*******************************************************
//pwm control for wheels
#define wheel_PWM 21     //All wheels will have the same speed, thus only one pwm control
#define FR_BL_PWM_Pin 21 //Front Right & Back Left wheels should be at the same speed
#define BR_FL_PWM_Pin 22  //Back Right & Front Left wheels should be at the same speed
//direction control for wheels
#define wheel_DIR_LF_1 19
#define wheel_DIR_LF_2 18
#define wheel_DIR_LB_1 5
#define wheel_DIR_LB_2 17
#define wheel_DIR_RF_1 16
#define wheel_DIR_RF_2 4
#define wheel_DIR_RB_1 0
#define wheel_DIR_RB_2 2
//pins for angle measurement
#define theta_Pin 27
#define phi_Pin 25

//xz plane
//PID XZ CONSTSNATAS
#define PID_XZ_MIN -180
#define PID_XZ_MAX 180
#define PID_XZ_SAMPLE_MILLI 20//20  //Sample Time of 20 is perfect!
#define SETPOINT_PHI 0

#define MIN_ABS_SPEED 0
double setpointPhi=SETPOINT_PHI;


double phiAngle=0;
double phiPID =0;
double phiAngleCopy=0;
double phiPIDCopy =0;

#define PID_XZ_KP 1500//2500//1500 is the magic number
#define PID_XZ_KI 2000//3000//2000 is as well
#define PID_XZ_KD 10//10 do be

PID xz_PID(&phiAngle,&phiPID,&setpointPhi,PID_XZ_KP,PID_XZ_KI,PID_XZ_KD,DIRECT);


//yz plane
//PID XZ CONSTSNATAS
#define PID_YZ_MIN -180
#define PID_YZ_MAX 180
#define PID_YZ_SAMPLE_MILLI 20//20  //Sample Time of 20 is perfect!
#define SETPOINT_THETA 0


double setpointTheta=SETPOINT_THETA;


double thetaAngle=0;
double thetaPID =0;
double thetaAngleCopy=0;
double thetaPIDCopy =0;

#define PID_YZ_KP 2750// 2000 is the magic number
#define PID_YZ_KI 3000//3300 is as well
#define PID_YZ_KD 15//15 is as

PID yz_PID(&thetaAngle,&thetaPID,&setpointTheta,PID_YZ_KP,PID_YZ_KI,PID_YZ_KD,DIRECT);

//*******************************************************
//constants
//*******************************************************
const int PWMfreq = 100;
const int PWMchannel_FR_BL = 1; //try change it to 2 if lag occurs
const int PWMchannel_BR_FL = 2; //try change it to 2 if lag occurs
const int PWMres = 8;
//*******************************************************
//Multiprocessing eeeek
//*******************************************************
TaskHandle_t Control;
TaskHandle_t FileHandle;
//*******************************************************
//File Writing eeeek
//*******************************************************
File file;
unsigned long iniTime;
volatile bool buttonPress = false;
const int buttonPin = 0;
double bufferFile[2][250];
int indexBuffer=0;
int indexWrite=0;
int stopWrite=0;
bool stopCar=false;


void writeHandle() {

    phiAngleCopy = phiAngle;
    phiPIDCopy = phiPID;
    thetaAngleCopy = thetaAngle;
    thetaPIDCopy = thetaPID;

}

void writeFile()
{

if(((millis()-iniTime)>25000)&&((millis()-iniTime)<50000))
{
    file = LittleFS.open("/ds_plan.txt", "a");
    file.print(phiAngleCopy);
    file.print('\t');
    file.print(phiPIDCopy);
    file.print('\t');
    file.print(thetaAngleCopy);
    file.print('\t');
    file.println(thetaPIDCopy);
    file.close();
    
    /*file = LittleFS.open("/ds_plan.txt", "a");
    file.print(bufferFile[0][indexWrite]);
    file.print('\t');
    file.println(bufferFile[1][indexWrite]);
    file.close();    

    indexWrite=indexWrite+1;

    if(indexWrite==stopWrite)
    {
      stopCar=true;
    }
    
    if(indexWrite>999)
    {
      indexWrite=0;
    }*/
    stopCar=false;
}else if (((millis()-iniTime)>50000)||((millis()-iniTime)<20000))
{
  stopCar=true;
}
  
}
void setupFile(const char* filename)
{
  //pinMode(buttonPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, RISING);
  
  LittleFS.begin();
  /*file = LittleFS.open("/ds_plan.txt", "a");
  LittleFS.format();
  file = LittleFS.open(filename, "w");
  file.print("Phi(rad)-> ");
  file.print('\t');
  file.println("PID(8 bit)->");
  file.close();//*/
}

void PID_Setup()
{
  xz_PID.SetOutputLimits(PID_XZ_MIN,PID_XZ_MAX);
  xz_PID.SetMode(AUTOMATIC);
  xz_PID.SetSampleTime(PID_XZ_SAMPLE_MILLI);//*/

  yz_PID.SetOutputLimits(PID_YZ_MIN,PID_YZ_MAX);
  yz_PID.SetMode(AUTOMATIC);
  yz_PID.SetSampleTime(PID_YZ_SAMPLE_MILLI);//*/
}
void Motor_Setup()
{
   //Wheel pwm setup
  //pinMode(wheel_PWM,OUTPUT); 

  //Wheel speed pwm setupt
  pinMode (FR_BL_PWM_Pin,OUTPUT);
  pinMode (BR_FL_PWM_Pin,OUTPUT);
  //Wheel dir setup
  pinMode (wheel_DIR_LF_1,OUTPUT);
  pinMode (wheel_DIR_LF_2,OUTPUT);
  pinMode (wheel_DIR_LB_1,OUTPUT);
  pinMode (wheel_DIR_LB_2,OUTPUT);
  pinMode (wheel_DIR_RF_1,OUTPUT);
  pinMode (wheel_DIR_RF_2,OUTPUT);
  pinMode (wheel_DIR_RB_1,OUTPUT);
  pinMode (wheel_DIR_RB_2,OUTPUT);

    //PWM Motor control
  ledcSetup(PWMchannel_FR_BL, PWMfreq, PWMres);
  ledcAttachPin(FR_BL_PWM_Pin, PWMchannel_FR_BL);
  
  ledcSetup(PWMchannel_BR_FL, PWMfreq, PWMres);
  ledcAttachPin(BR_FL_PWM_Pin, PWMchannel_BR_FL);

  balance(0);
}
void ControlCode(void * parameter) {
  for (;;) {
    
    phiAngle = get_Phi();
    thetaAngle=get_Theta();
    xz_PID.Compute();
    yz_PID.Compute();
    //
    //balance(thetaPID);

    //writeHandle();
  
    crawl(thetaPID,phiPID);

    delay(1000);

    
  }
}
void FileCode(void * parameter) {
  for (;;) {
   //Serial.println(millis());
    //writeFile();
   //Serial.println(millis());
    
  }
}
void setup() {
  
 
  //Serial Communication is starting with 115200 of baudrate speed

  
   
  Motor_Setup();

  PID_Setup();

  setupFile("/ds_plan.txt");

  
  
  Serial.begin(115200);
  
  delay(5000);

  iniTime=millis();

  xTaskCreatePinnedToCore(
      ControlCode, /* Function to implement the task */
      "Control", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Control,  /* Task handle. */
      0); /* Core where the task should run */

    xTaskCreatePinnedToCore(
      FileCode, /* Function to implement the task */
      "FileHandle", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &FileHandle,  /* Task handle. */
      1); /* Core where the task should run */
  

  
}
void loop() {

  

      
  //writeFile();
}


double get_Phi()
{
  int phi_ADC=analogRead(phi_Pin);
  double phi_degree=(phi_ADC*(0.0603)-127.77)-3.5/*3.5*/;
  double rad= (phi_degree*PI)/180;
  /*Serial.print(" Phi Angle in degree: ");
  Serial.print(phi_degree);
  Serial.print(" and in adc ");
  Serial.print(phi_ADC);//*/
  return rad;
}
double get_Theta()
{
  int theta_ADC=analogRead(theta_Pin);
 /* double theta_degree;
if (theta_ADC<1950)
{
  theta_degree=(theta_ADC*(0.0612)-112.925);
}
else if (theta_ADC > 2040)
{
  theta_degree=(theta_ADC*(0.0769)-160.769);
}else
{
  theta_degree=0;
}//*/
  
  double theta_degree=(theta_ADC*(0.0861)-44.137)+3.25/*3.75*/;
  double rad= (theta_degree*PI)/180;
  /*Serial.print(" Theta Angle in degree: ");
  Serial.print(theta_degree);
  Serial.print(" and in ADC ");
  Serial.println(theta_ADC);//*/
  return rad;
}

void balance(int speed_Y)
{
  /*
  if (speed_X<0)
  {
      digitalWrite(wheel_DIR_LB_1,1);
      digitalWrite(wheel_DIR_LB_2,0);
      digitalWrite(wheel_DIR_RF_1,1);
      digitalWrite(wheel_DIR_RF_2,0); 
      digitalWrite(wheel_DIR_RB_1,1);
      digitalWrite(wheel_DIR_RB_2,0);
      digitalWrite(wheel_DIR_LF_1,1);
      digitalWrite(wheel_DIR_LF_2,0); 
  }else if (speed_X>=0)
  {


      digitalWrite(wheel_DIR_LB_1,0);
      digitalWrite(wheel_DIR_LB_2,1);
      digitalWrite(wheel_DIR_RF_1,0);
      digitalWrite(wheel_DIR_RF_2,1);
      digitalWrite(wheel_DIR_RB_1,0);
      digitalWrite(wheel_DIR_RB_2,1);
      digitalWrite(wheel_DIR_LF_1,0);
      digitalWrite(wheel_DIR_LF_2,1);
  }

  speed_X=abs(speed_X)+MIN_ABS_SPEED;
  speed_X=constrain(speed_X,MIN_ABS_SPEED,180);

  if(stopCar)
  {
    speed_X=0;
  }

  ledcWrite(PWMchannel_FR_BL,speed_X);
  ledcWrite(PWMchannel_BR_FL,speed_X);//*/
  
  
  if (speed_Y<0)
  {
      digitalWrite(wheel_DIR_LB_1,1);
      digitalWrite(wheel_DIR_LB_2,0);
      digitalWrite(wheel_DIR_RF_1,1);
      digitalWrite(wheel_DIR_RF_2,0); 
      digitalWrite(wheel_DIR_RB_1,0);
      digitalWrite(wheel_DIR_RB_2,1);
      digitalWrite(wheel_DIR_LF_1,0);
      digitalWrite(wheel_DIR_LF_2,1); 
  }else if (speed_Y>=0)
  {


      digitalWrite(wheel_DIR_LB_1,0);
      digitalWrite(wheel_DIR_LB_2,1);
      digitalWrite(wheel_DIR_RF_1,0);
      digitalWrite(wheel_DIR_RF_2,1);
      digitalWrite(wheel_DIR_RB_1,1);
      digitalWrite(wheel_DIR_RB_2,0);
      digitalWrite(wheel_DIR_LF_1,1);
      digitalWrite(wheel_DIR_LF_2,0);
  }

  speed_Y=abs(speed_Y)+MIN_ABS_SPEED;
  speed_Y=constrain(speed_Y,MIN_ABS_SPEED,180);

  if(stopCar)
  {
    speed_Y=0;
  }

  ledcWrite(PWMchannel_FR_BL,speed_Y);
  ledcWrite(PWMchannel_BR_FL,speed_Y);
  //*/
}
void crawl(int speed_X,int speed_Y)//The two calculated PID Values
{
  double angle =atan2 (speed_Y,speed_X);
  //Serial.println(angle);
  double spd = sqrt (speed_X*speed_X+speed_Y*speed_Y);
  
  //Sin functions control direction and te appropriate speed
  float FR_BL_data=sin((angle)+0.25*PI);
  float BR_FL_data=sin((angle)+0.75*PI);


  if(FR_BL_data<0){//Front Right and Back Left Wheels go forward
      digitalWrite(wheel_DIR_LB_1,1);
      digitalWrite(wheel_DIR_LB_2,0);
      digitalWrite(wheel_DIR_RF_1,1);
      digitalWrite(wheel_DIR_RF_2,0); 
  }
  else {//Front Right and Back left Wheels go backward
      digitalWrite(wheel_DIR_LB_1,0);
      digitalWrite(wheel_DIR_LB_2,1);
      digitalWrite(wheel_DIR_RF_1,0);
      digitalWrite(wheel_DIR_RF_2,1); 
  }

  if(BR_FL_data>0){//Back Right and Front Left wheels go forward
      digitalWrite(wheel_DIR_RB_1,1);
      digitalWrite(wheel_DIR_RB_2,0);
      digitalWrite(wheel_DIR_LF_1,1);
      digitalWrite(wheel_DIR_LF_2,0);
  }
  else{//Back Right and Front Left wheels go backward
      digitalWrite(wheel_DIR_RB_1,0);
      digitalWrite(wheel_DIR_RB_2,1);
      digitalWrite(wheel_DIR_LF_1,0);
      digitalWrite(wheel_DIR_LF_2,1);
  }

  float mag_FR_BL=abs(FR_BL_data);
  float mag_BR_FL=abs(BR_FL_data);
  //Have to map this 255 relating to a force
  
  double FR_BL_speed=map(100*mag_FR_BL,0,100,MIN_ABS_SPEED,spd);
  double BR_FL_speed=map(100*mag_BR_FL,0,100,MIN_ABS_SPEED,spd);

    if(stopCar)
    {
    FR_BL_speed=0;
    BR_FL_speed=0;
    }

  ledcWrite(PWMchannel_FR_BL,FR_BL_speed);
  ledcWrite(PWMchannel_BR_FL,BR_FL_speed);
  
}
