#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>


ros::NodeHandle  motor;

geometry_msgs::Twist motor_data;

int scaling(float speed)
{
  float scale_speed;
  scale_speed = ((255*speed)/65); 
  return int(scale_speed);
}

int direc(float speed)
{
  // 0 = Forward
  // 1 = Backward
  
  if(speed>=0)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

void cal_data(const geometry_msgs::Twist& sub_data)
{
  motor_data = sub_data;
  
  motor_data.linear.x = scaling(motor_data.linear.x);
  motor_data.angular.x = direc(motor_data.linear.x);
  motor_data.linear.z = scaling(motor_data.linear.z);
  motor_data.angular.z = direc(motor_data.linear.z);
  
  analogWrite(3, motor_data.linear.x); 
  digitalWrite(2, motor_data.angular.x); 
  analogWrite(5, motor_data.linear.z);
  digitalWrite(4, motor_data.angular.z);  
}

ros::Subscriber<geometry_msgs::Twist> mod_data("mod_msg", &cal_data);

void setup() {
  motor.getHardware()->setBaud(57600);
  motor.initNode();
  motor.subscribe(mod_data);
  pinMode(3, OUTPUT); //motor Driver-1 Speed Left
  pinMode(2, OUTPUT); //motor Driver-1 Direction Left
  pinMode(5, OUTPUT); //motor Driver-2 Speed Right
  pinMode(4, OUTPUT); //motor Driver-2 Direction Right
  
}

void loop() {
  motor.spinOnce();
  delay(10);
  
}