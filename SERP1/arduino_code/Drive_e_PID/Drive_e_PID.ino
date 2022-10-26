#include <RunningAverage.h>
#include <ros.h>
#include <ArduPID.h>
#include <serp/RobotInfo.h>
#include <serp/Velocity.h>

//max rpm=95rpm


//MOTOR RIGHT->A->1
//MOTOR LEFT->B->2

RunningAverage myRA1(50);
RunningAverage myRA2(50);

#define encoder_pulse   13
#define gear_ratio      120
#define wheel_diameter  0.069
#define pi 3.1415926
#define LOOPTIME 1
#define AIN1 4 //right 
#define BIN1 8 //left
#define AIN2 7 //right
#define BIN2 10 //left
#define PWMA 6 //right
#define PWMB 5 //left
#define STBY 9
#define ADCB A0
#define encodPinA1      3     //right
#define encodPinB1      13  //right
#define encodPinA2      2   //left              
#define encodPinB2      12 //left

ArduPID right_motor;
ArduPID left_motor;


// Global variables
ros::NodeHandle  nh;

double setpoint_r;
double setpoint_l;





double rpm_act1;
double output_r;
double rpm_act2;
double output_l;

double p = 20 ;
double i = 0;
double d = 0;

double v1Filt = 0;
double v1Prev = 0;

unsigned long lastMilli = 0;
volatile long count1 = 0;          
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;

char vel1[4];
char vel2[4];
// Calback que é chamada sempre que chegam novos dados de velocidade para os motores
// Atualiza o valor da velocidade guardado nas variáveis globais
void cb_velocity(const serp::Velocity& msg) 
{
 
  setpoint_l =msg.vel_motor_left/2;
  setpoint_r =msg.vel_motor_right/2;

}

// Tipo de mensagem a usar para publicar o estado atual do robô (bateria + velocidade + delta Pose) 
serp::RobotInfo msg_hardware_state;

// ROS Publisher/Subscriber
ros::Publisher publisher("hardware_info", &msg_hardware_state);
ros::Subscriber<serp::Velocity> subscriber("motors_vel", &cb_velocity);


void setup() {
  Serial.begin(57600);

  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), encoder1, RISING);

  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), encoder2, RISING);

  //rosserial setup
  nh.initNode();
  nh.advertise(publisher);
  nh.subscribe(subscriber);

//  //setup PID
  right_motor.begin(&rpm_act1, &output_r, &setpoint_r, p, i, d);
  left_motor.begin(&rpm_act2, &output_l, &setpoint_l, p, i, d);

}

void loop() 
{
  int val;
  float battery_perc_ori;
  float battery_perc;
  val=analogRead(ADCB);
  battery_perc_ori=val/1023.0*5.0;
  battery_perc=mapping(battery_perc_ori, 4.5 ,5.1 , 0, 100);
  double lbatt_state;

  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   
  {      
    //get speeds
    getMotorData(time-lastMilli);
 
    //compute PID
    right_motor.compute();
    left_motor.compute();
    
    //set motor speed                                          
    motorspeed(output_r,1);
    motorspeed(output_l*1.4,2);
    myRA1.addValue(rpm_act1);
    myRA2.addValue(rpm_act2);
    // Publish info about actual robot state
    lbatt_state=msg_hardware_state.battery_level;
    msg_hardware_state.battery_level = battery_perc;
    msg_hardware_state.vel_linear = myRA2.getAverage(); //left motor
    msg_hardware_state.vel_angular = myRA1.getAverage(); //right motor

    if(lbatt_state!=msg_hardware_state.battery_level)
    publisher.publish(&msg_hardware_state);
                                         
    lastMilli = time;    
  }


 
nh.spinOnce();
 

}

void getMotorData(unsigned long time)  
{
  
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio); //right
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio); //left
 countAnt1 = count1;
 countAnt2 = count2;
 
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}

void motorspeed(double output, int id) {
   if (id==1) //right motor
   {
      if (output>=0) //forward
      {
        digitalWrite(AIN1,HIGH); 
        digitalWrite(AIN2,LOW);
        analogWrite(PWMA,output_r);      
      }
      if (output<0) //backward
      {
        digitalWrite(AIN1,LOW); 
        digitalWrite(AIN2,HIGH);
        analogWrite(PWMA,abs(output_r));
      }
   }
   else  //left motor
    {
      if (output<=0) //forward
      {
        //Serial.println("motor right");
        //Serial.println(output_l);
        digitalWrite(BIN1,HIGH); 
        digitalWrite(BIN2,LOW);
        analogWrite(PWMB,abs(output_l));   
           
      }
      if (output>0) //backward
      {
        digitalWrite(BIN1,LOW); 
        digitalWrite(BIN2,HIGH);
        analogWrite(PWMB,abs(output_l));
      }
   }
}

float mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
