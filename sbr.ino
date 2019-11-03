
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
//MPU6050 mpu6050(Wire, 0.1, 0.9);

const int pwm_pin[2] = {9,10}; 
const int directA_pin[2] = {7,2};
const int directB_pin[2] = {5,4};    

float pid=0;
float lastpid=0;

float kp=17,kd=1.3,ki=0.1;

float angleLimit=15;

float setPoint=2.1;

int timeSampling=1;

float speedMax=220;

float offsetValue=1;

float nowError=0;

float lastError=0;

float valueX=0;

float angleSpeedX=0;
float accX=0;


float interval=0;
float preInterval=0;

float errKd=0;
float errKi=0;


int nowTime=0;
int lastTime=0;


void mundur_all(int value){
  digitalWrite(directA_pin[0], LOW);//kanan
  digitalWrite(directA_pin[1], HIGH);

  digitalWrite(directB_pin[0], HIGH);//kiri
  digitalWrite(directB_pin[1], LOW);

  analogWrite(pwm_pin[0],value);//kanan
  analogWrite(pwm_pin[1],value);//kiri
}

void stop_all(){
  digitalWrite(directA_pin[0],HIGH);
  digitalWrite(directA_pin[1],HIGH);

  digitalWrite(directB_pin[0],HIGH);
  digitalWrite(directB_pin[1],HIGH);

  analogWrite(pwm_pin[0],0);
  analogWrite(pwm_pin[1],0);
}

void maju_all(int value){
  digitalWrite(directA_pin[0], HIGH);//kanan
  digitalWrite(directA_pin[1], LOW);

  digitalWrite(directB_pin[0], LOW);//kiri
  digitalWrite(directB_pin[1], HIGH);

  analogWrite(pwm_pin[0],value);//kanan
  analogWrite(pwm_pin[1],value);//kiri
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);

  // initialize the LED pin as an output:
  pinMode(pwm_pin[0], OUTPUT);
  pinMode(pwm_pin[1], OUTPUT);
  pinMode(directA_pin[0], OUTPUT);
  pinMode(directA_pin[1], OUTPUT);
  pinMode(directB_pin[0], OUTPUT);
  pinMode(directB_pin[1], OUTPUT);
  stop_all();

  TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x00;
  ICR1L=0x00;
  OCR1AH=0x00;
  OCR1AL=0x00;
  OCR1BH=0x00;
  OCR1BL=0x00;

}

void loop() {
  
  mpu6050.update();
  
  valueX=mpu6050.getAngleX();
  //Serial.print("angleX : ");Serial.println(valueX);

  //define error  

  nowTime=millis();
  interval = (nowTime - preInterval) * 0.001;
  
  if(valueX<(setPoint+offsetValue) && valueX>(setPoint-offsetValue)){
    nowError=0;
  }
  else{
    nowError=(setPoint)-valueX;
  }

  errKd=(nowError-lastError);
  errKi=(errKi+nowError);

  preInterval = nowTime;

  pid=((kp)*nowError)+(kd*errKd/interval)+(ki*errKi*interval);
  pid=abs(pid);
  
  
  lastError=nowError;

  if(pid>speedMax){
    pid=speedMax;
  }
  else if(pid<(speedMax*0.25)){
    pid=speedMax*0.25;
  }
  
  
  if(nowError>0){
    
    if(pid!=lastpid){
      maju_all(pid);
    }

  }
  else if(nowError<0){
    
    if(pid!=lastpid){
      mundur_all(pid);
    }

  }
  else{
    if(pid!=lastpid){
      stop_all();
    }
  }
  
  
  lastpid=pid;

}
