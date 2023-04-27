#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// Motor A connections
const int pinPwmA = 3;
const int pinDirA = 2;

// Motor B connections
const int pinPwmB = 6;
const int pinDirB = 7;

ros::NodeHandle nh;
std_msgs::String debug_msg;

ros::Publisher debug_pub("arduino_debug", &debug_msg);

const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];

void drive_callback(const geometry_msgs::Twist& cmd_vel) {
  float left_motor_speed = -1.0 * (cmd_vel.linear.x - cmd_vel.angular.z);
  float right_motor_speed = (cmd_vel.linear.x + cmd_vel.angular.z);

  char left_motor_str[10];
  char right_motor_str[10];

  dtostrf(left_motor_speed, 5, 2, left_motor_str);
  dtostrf(right_motor_speed, 5, 2, right_motor_str);

  char buffer[50];
  sprintf(buffer, "Left motor: %s, Right motor: %s", left_motor_str, right_motor_str);
  debug_msg.data = buffer;
  debug_pub.publish(&debug_msg);

  int left_motor_pwm = abs(int(left_motor_speed * 100));
  int right_motor_pwm = abs(int(right_motor_speed * 100));

  if (left_motor_speed > 0) {
    digitalWrite(pinDirA, LOW);
  } else {
    digitalWrite(pinDirA, HIGH);
  }

  if (right_motor_speed > 0) {
    digitalWrite(pinDirB, LOW);
  } else {
    digitalWrite(pinDirB, HIGH);
  }

  analogWrite(pinPwmA, left_motor_pwm);
  analogWrite(pinPwmB, right_motor_pwm);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", drive_callback);

void setup() {
  Serial.begin(57600);

  pinMode(pinPwmA, OUTPUT);
  pinMode(pinDirA, OUTPUT);
  pinMode(pinPwmB, OUTPUT);
  pinMode(pinDirB, OUTPUT);

  digitalWrite(pinDirA, LOW);
  digitalWrite(pinDirB, LOW);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug_pub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}