#include <FastLED.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

int RPWM_Left=3;
int LPWM_Left=4;
int RPWM_Right=7;
int LPWM_Right=8;
// timer 0
int L_EN_Left=5;
int R_EN_Left=6;
int L_EN_Right=9;
int R_EN_Right=10;

#define NUM_LEDS 15
#define LED_PIN 11

CRGB led[NUM_LEDS];

int buzzer = 12;     // the pin that the LED is atteched to
int sensor = 22;  // the pin that the sensor is atteched to
int sensor1 = 23;
int sensor2 = 24;
int sensor3 = 25;
int state = LOW;             // by default, no motion detected
int LEFT,FRONT,RIGHT,BACK = 0;                 // variable to store the sensor status (value)

ros::NodeHandle node;
geometry_msgs::Twist msg;

void Red() {

FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);

  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(225, 0, 0);    
    }
    
}

void Green() {

FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);

  for (int i = 0; i < NUM_LEDS; i++){
    led[i] = CRGB(0, 225, 0);    
    }
    
}
void forward(){
  digitalWrite(R_EN_Left,HIGH);
  digitalWrite(L_EN_Left,HIGH);
  digitalWrite(R_EN_Right,HIGH);
  digitalWrite(L_EN_Right,HIGH);

 analogWrite(LPWM_Left,255);
 analogWrite(LPWM_Right,255);

  }
void left(){
  digitalWrite(R_EN_Left,HIGH);
  digitalWrite(L_EN_Left,HIGH);
  digitalWrite(R_EN_Right,HIGH);
  digitalWrite(L_EN_Right,HIGH); 

 analogWrite(LPWM_Left,255);
 analogWrite(LPWM_Right,0);

 
  }
 void right(){
  digitalWrite(R_EN_Left,HIGH);
  digitalWrite(L_EN_Left,HIGH);
  digitalWrite(R_EN_Right,HIGH);
  digitalWrite(L_EN_Right,HIGH);
  
  analogWrite(LPWM_Left,0);
 analogWrite(LPWM_Right,255);
 
  }
void back(){
  digitalWrite(R_EN_Left,HIGH);
  digitalWrite(L_EN_Left,HIGH);
  digitalWrite(R_EN_Right,HIGH);
  digitalWrite(L_EN_Right,HIGH);
  
  analogWrite(RPWM_Left,255);
 analogWrite(RPWM_Right,255);
  }
void hold(){
  digitalWrite(R_EN_Left,LOW);
  digitalWrite(L_EN_Left,LOW);
  digitalWrite(R_EN_Right,LOW);
  digitalWrite(L_EN_Right,LOW);
  }
  

void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{

  if(cmd_vel.linear.x > 0 && cmd_vel.angular.z == 0)
  {
    forward(); //i
    analogWrite(test, 255);
  }
  else
  {
    if(cmd_vel.linear.x == 0 && cmd_vel.angular.z > 0)
    {
      left(); //j
      analogWrite(test, 255);
    }
    else
    {
      if(cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0)
      {
        hold(); //k
        analogWrite(test, 0);
      }
      else
      {
        if(cmd_vel.linear.x == 0 && cmd_vel.angular.z < 0)
        {
          right(); //l
          analogWrite(test, 255);
        }
        else
        {
          if(cmd_vel.linear.x < 0 && cmd_vel.angular.z == 0)
          {
            back(); //,
            analogWrite(test, 255);
          }
          else
          {
            hold(); //default
          }
        }
      }
    }
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", roverCallBack);


void setup() {
//  Serial.begin(57600);
//  BT1.begin(57600);  
  node.initNode();
  node.subscribe(sub);
  
  pinMode(led, OUTPUT);      // initalize LED as an output
  pinMode(sensor, INPUT);    // initialize sensor as an input
  Serial.begin(9600);        // initialize serial
  
  for(int i=5;i<9;i++){
   pinMode(i,OUTPUT);
  }
  
}

void loop(){
  LEFT = digitalRead(sensor);   // read sensor value
  FRONT = digitalRead(sensor1);
  RIGHT = digitalRead(sensor2);
  BACK = digitalRead(sensor3);
  if (LEFT == HIGH || FRONT == HIGH || RIGHT == HIGH || BACK == HIGH) {           // check if the sensor is HIGH
    digitalWrite(buzzer, HIGH);   // turn LED ON
    delay(100);                // delay 100 milliseconds 
    Red();
    delay(100);
    FastLED.show();
    hold();
    
    if (state == LOW) {
      Serial.println("Motion detected!"); 
      state = HIGH;       // update variable state to HIGH
    }
  } 
  else {
      digitalWrite(buzzer, LOW); // turn LED OFF
      delay(200);             // delay 200 milliseconds 
      Green();
      delay(100);
      FastLED.show();
      
      if (state == HIGH){
        Serial.println("Motion stopped!");
        state = LOW;       // update variable state to LOW
    }
  }
}
