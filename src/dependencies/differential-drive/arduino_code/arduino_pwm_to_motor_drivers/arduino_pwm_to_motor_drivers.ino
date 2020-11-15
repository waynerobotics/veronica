#include <ros.h>
#include <std_msgs/Float32.h>

#define PWM_RIGHT 5
#define DIR_RIGHT 6
#define PWM_LEFT 7
#define DIR_LEFT 8
#define MIN_PWM 50
#define MAX_PWM 150
#define PWM_CHANGE_INCREMENT 2

ros::NodeHandle nh;

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

//this section increments PWM changes instead of jarring/dangerous sudden big changes
int get_new_pwm_out(int requestedPwm, int lastPwm)
{

    if (is_direction_change(requestedPwm, lastPwm))
        requestedPwm = 0;

    if (abs(requestedPwm) > lastPwm)
        return lastPwm + PWM_CHANGE_INCREMENT;
    else if (abs(requestedPwm) < lastPwm)
        return lastPwm - PWM_CHANGE_INCREMENT;
    else
        return requestedPwm;
}

//this happens when right wheel cmd message is recieved
void rightMessageCb(const std_msgs::Float32 &request_pwm)
{

    //this blinks the onboard LED for debugging. One blink per pwm value of 10.
    //DO NOT try to run robot witht his uncommented, as it adds significant delay before updating pwm output
    //can verify rosserial connection and that arduino is recieving topics with command line commend:
    //rostopic pub -1 rmotor_pwm_cmd std_msgs/Float32 150   (where last argument is a pwm value from 0-255)
    // for (int i = 0; i <  static_cast<int>(request_pwm.data) / 10; i++)
    // {
    //     delay(300);
    //     digitalWrite(LED_BUILTIN, HIGH);
    //     delay(300);
    //     digitalWrite(LED_BUILTIN, LOW);
    // }

    static int lastPwm = 0;
    int requestedPwm = (int)request_pwm.data;

    //set direction pins - invert get_requested_dir() output if motor runs backwards
    digitalWrite(DIR_RIGHT, get_requested_direction(requestedPwm));

    //update pwm value that will actually be written
    lastPwm = get_new_pwm_out(requestedPwm, lastPwm);

    //actually output pwm values
    analogWrite(PWM_RIGHT, lastPwm);
}

//this happens when left wheel cmd message is recieved
void leftMessageCb(const std_msgs::Float32 &request_pwm)
{
    static int lastPwm = 0;
    int requestedPwm = (int)request_pwm.data;

    //set direction pins - invert get_requested_dir() output if motor runs backwards
    digitalWrite(DIR_LEFT, get_requested_direction(requestedPwm));

    //update pwm value that will actually be written
    lastPwm = get_new_pwm_out(requestedPwm, lastPwm);

    //actually output pwm values
    analogWrite(PWM_LEFT, lastPwm);
}

//subscribe to cmd_vel
ros::Subscriber<std_msgs::Float32> subRight("rmotor_pwm_cmd", &rightMessageCb);
ros::Subscriber<std_msgs::Float32> subLeft("lmotor_pwm_cmd", &leftMessageCb);

void setup()
{
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

    pinMode(PWM_RIGHT, OUTPUT);
    pinMode(DIR_RIGHT, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(DIR_LEFT, OUTPUT);
    analogWrite(PWM_LEFT, 0);
    analogWrite(PWM_RIGHT, 0);
    digitalWrite(DIR_LEFT, 0);
    digitalWrite(DIR_RIGHT, 0);

    nh.initNode();
    nh.subscribe(subRight);
    nh.subscribe(subLeft);
}

void loop()
{
    nh.spinOnce();
    delay(20);
}
