/*
  ros_pwm_handler.ino is a ROS (Robot Operating System) node that runs on
  an arduino nano and subscribes to a pair of Float32 messages (as published for example, by the differential drive package -
  http://wiki.ros.org/differential_drive) and in turn sends that out as a PWM signal to the motor drivers.

  Lloyd Brombach, November 2020
  lbrombach2@gmail.com
*/

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#define PWM_RIGHT 5 //6   // PWM output pin for right motor
#define DIR_RIGHT 3 // 3   // Direction output pin for right motor

#define PWM_LEFT 6 //5    // PWM output pin for left motor
#define DIR_LEFT 2 //2    // Direction output pin for left motor
#define MIN_PWM 50    //program outputs zero below this value to avoid energized motor below the movement threshold
#define MAX_PWM 150   //adjust to set max speed
#define PWM_CHANGE_INCREMENT 2  //adjust to change how quickly output changes

#define LIGHT_PIN A2 

int left_cmd = 0;
int right_cmd = 0;

volatile unsigned long lastLightToggle = 0;
bool lightOn = true;


ros::NodeHandle nh;

std_msgs::Int16 left;
std_msgs::Int16 right;
ros::Publisher pubLeft("left", &left);

bool get_requested_direction(int requested_pwm)
{
  return (requested_pwm > 0) ? 1 : 0;
}

//if PwmReq*PwmOut is negative, that means the wheel is switching
//directions and we should bring to a stop before switching directions
bool is_direction_change(int requested_pwm, int last_pwm)
{
  if (requested_pwm * last_pwm < 0)
    return true;
  else
    return false;
}

//this section increments PWM changes instead of making jarring/dangerous sudden big changes
int get_new_pwm_out(int requestedPwm, int lastPwm)
{
  int newPWM = 0;
  if (is_direction_change(requestedPwm, lastPwm))
    requestedPwm = 0;

  if (requestedPwm > lastPwm)
  {
    newPWM = lastPwm + PWM_CHANGE_INCREMENT;
  }
  else if (requestedPwm < lastPwm)
  {
    newPWM = lastPwm - PWM_CHANGE_INCREMENT;
  }
  else
    newPWM = requestedPwm;

  return newPWM;
}
//this happens when right wheel cmd message is recieved
void rightMessageCb(const std_msgs::Float32 &request_pwm)
{
  right_cmd = (int)request_pwm.data;
}

//this happens when left wheel cmd message is recieved
void leftMessageCb(const std_msgs::Float32 &request_pwm)
{
  left_cmd = (int)request_pwm.data;
}



void autoCallback(bool &autoIsActive){
  if(autoIsActive && millis() - lastLightToggle > 1500){
    lightOn = !lightOn;
    digitalWrite(LIGHT_PIN, lightOn);
    lastLightToggle = millis();
  }
  else{
    digitalWrite(LIGHT_PIN, 0);    
  }
}

//subscribe to cmd_vel
ros::Subscriber<std_msgs::Float32> subRight("rmotor_pwm_cmd", &rightMessageCb);
ros::Subscriber<std_msgs::Float32> subLeft("lmotor_pwm_cmd", &leftMessageCb);
ros::Subscriber<std_msgs::Bool> subAutonomousModeActive("autonomous_active", &autoCallback);

void setup()
{
  // Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);
  delay(500);
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(LED_BUILTIN, 1);
  delay(500);
  digitalWrite(LED_BUILTIN, 0);

  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, 0);

  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
  digitalWrite(DIR_LEFT, 0);
  digitalWrite(DIR_RIGHT, 0);

 // Serial.begin(9600);
 // nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(subRight);
  nh.subscribe(subLeft);
nh.advertise(pubLeft);
}



void loop()
{
  nh.spinOnce();

  static int lastLeft = 0;
  static int lastRight = 0;

  left_cmd = (is_direction_change(left_cmd, lastLeft))    ? 0 : left_cmd;
  right_cmd = (is_direction_change(right_cmd, lastRight)) ? 0 : right_cmd;


  //update pwm value that will actually be written
  int nextLeft = get_new_pwm_out(left_cmd, lastLeft);
  int nextRight = get_new_pwm_out(right_cmd, lastRight);

  left.data = nextLeft;
  pubLeft.publish(&left);

  //set direction pins - invert get_requested_dir() output if motor runs backwards
  if(abs(nextLeft) <= PWM_CHANGE_INCREMENT)
  {
    digitalWrite(DIR_LEFT, get_requested_direction(nextLeft));
  }
    if(abs(nextRight) <= PWM_CHANGE_INCREMENT)
  {
    digitalWrite(DIR_RIGHT, get_requested_direction(nextRight));
  }  
  


  //actually output pwm values
  analogWrite(PWM_LEFT, abs(nextLeft));
  analogWrite(PWM_RIGHT, abs(nextRight));

  //analogWrite(PWM_LEFT, 0);

  lastLeft = nextLeft;
  lastRight = nextRight;

}
