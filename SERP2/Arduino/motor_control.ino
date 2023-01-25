#include <ros.h>

#include <ArduPID.h>
//#include "serp/RobotInfo.h"
#include "serp/Velocity.h"
//max rpm=95rpm


//MOTOR RIGHT->A->1
//MOTOR LEFT->B->2

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
#define ADCB 14
#define encodPinA1      3     //right
#define encodPinB1      13    //right
#define encodPinA2      2   //left  B 2          
#define encodPinB2      12 //left

ArduPID right_motor;
ArduPID left_motor;


// Global variables
ros::NodeHandle  nh;

double setpoint_r = 0;
double setpoint_l = 0;
double output_l;
double output_r;

// Calback que é chamada sempre que chegam novos dados de velocidade para os motores
// Atualiza o valor da velocidade guardado nas variáveis globais
void cb_velocity(const serp::Velocity &msg) {
  //nh.loginfo("Recebi novos dados de velocidade...\n"); // DEBUG -> APAGAR
  //setpoint_l = msg.vel_motor_left;
  //setpoint_r = msg.vel_motor_right;
  output_l = msg.vel_motor_left;
  output_r = msg.vel_motor_right;

}

// Tipo de mensagem a usar para publicar o estado atual do robô (bateria + velocidade + delta Pose) 
//serp::RobotInfo msg_hardware_state;

// ROS Publisher/Subscriber
//ros::Publisher publisher("hardware_info", &msg_hardware_state);
ros::Subscriber<serp::Velocity> subscriber("/vel", &cb_velocity);


double rpm_act1;

double rpm_act2;

double p = 10;
double i = 0;
double d = 0;

//double v1Filt = 0;
//double v1Prev = 0;

unsigned long lastMilli = 0;
volatile long count1 = 0;          
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;


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
  //nh.advertise(publisher);
  nh.subscribe(subscriber);

//  //setup PID
  right_motor.begin(&rpm_act1, &output_r, &setpoint_r, p, i, d);
  left_motor.begin(&rpm_act2, &output_l, &setpoint_l, p, i, d);

  right_motor.setOutputLimits(-120.0, 120.0);
  left_motor.setOutputLimits(-120.0, 120.0);

  right_motor.start();
  left_motor.start();

}

void loop() {
  int val;
 // float battery_perc_ori;
 // float battery_perc;
 // val=analogRead(ADCB);
 // battery_perc_ori=5.2;//val/1023.0*5.0;
 // battery_perc=map(battery_perc_ori, 4.5 ,5.1 , 0, 100);
  //Serial.println(battery_perc);
 // double lbatt_state;

  //analogWrite(PWMA,150); //Speed control of Motor A
  //analogWrite(PWMB,150); //Speed control of Motor B
  //control loop
  //setpoint_r=0;
  //setpoint_l=0;

  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      
    //get speeds
    getMotorData(time-lastMilli);
 
     //v1Filt = 0.854*v1Filt + 0.0728*rpm_act1 + 0.0728*v1Prev;
     //v1Prev = rpm_act1;
    //compute PID
    //right_motor.compute();
    //left_motor.compute();
    
    //set motor speed  
                                            
    motorspeed(output_r,1);
    motorspeed(output_l,2);

    // Publish info about actual robot state
  //  lbatt_state=msg_hardware_state.battery_level;
  //  msg_hardware_state.battery_level = battery_perc;
  //  msg_hardware_state.vel_linear = 3;
  //  msg_hardware_state.vel_angular = 0.1;
  //  msg_hardware_state.delta_pos_x = 2;
  //  msg_hardware_state.delta_pos_y = 1;
  //  msg_hardware_state.delta_orientation_z = 0.7;

    //if(lbatt_state!=msg_hardware_state.battery_level)
    //publisher.publish(&msg_hardware_state);
                                         
    lastMilli = time;    
  }
Serial.print("R  ");
Serial.println(output_r);
Serial.print("L  ");
Serial.println(output_l);

//Serial.println("ola");

// Serial.print("------");
// Serial.println(output_l);

 
nh.spinOnce();
 

}

void getMotorData(unsigned long time)  {
 //rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio); //right
 //rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio); //left
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time);
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
        //Serial.println("motor right");
        digitalWrite(AIN1,HIGH); 
        digitalWrite(AIN2,LOW);
        //Serial.println(output_r);
        analogWrite(PWMA,output_r);      
      }
      if (output<0) //backward
      {
        digitalWrite(AIN1,LOW); 
        digitalWrite(AIN2,HIGH);
        analogWrite(PWMA,-output_r);
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
        analogWrite(PWMB,-output_l);   
           
      }
      if (output>0) //backward
      {
        digitalWrite(BIN1,LOW); 
        digitalWrite(BIN2,HIGH);
        analogWrite(PWMB,output_l);
      }
   }
}