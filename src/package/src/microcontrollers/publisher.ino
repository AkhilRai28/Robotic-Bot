#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <HCSR04.h>

HCSR04 hcl(3, 2);   //Left Sensor
HCSR04 hcc(7, 6);   //Center Sensor (trig pin , echo pin)
HCSR04 hcr(5, 4); //Right Sensor

ros::NodeHandle  ultra;

geometry_msgs::Twist dis_data;
geometry_msgs::Twist motor_data;
ros::Publisher ultra_data("ultra_msg", &dis_data);

//int scaling(float speed)
//{
//  float scale_speed;
//  scale_speed = ((255*speed)/65);
//  return int(scale_speed);
//}
//
//int direc(float speed)
//{
//  // 0 = Forward
//  // 1 = Backward
//  
//  if(speed>=0)
//  {
//    return 0;
//  }
//  else
//  {
//    return 1;
//  }
//}
//
//
//
//void cal_data(const geometry_msgs::Twist& sub_data)
//{
//  motor_data = sub_data;
//  
//  motor_data.linear.x = scaling(motor_data.linear.x);
//  motor_data.angular.x = direc(motor_data.linear.x);
//  motor_data.linear.z = scaling(motor_data.linear.z);
//  motor_data.angular.z = direc(motor_data.linear.z);
//  
//  analogWrite(5, motor_data.linear.x); 
//  digitalWrite(4, motor_data.angular.x); 
//  analogWrite(9, motor_data.linear.z);
//  digitalWrite(8, motor_data.angular.z);  
//}
//
//ros::Subscriber<geometry_msgs::Twist> mod_data("mod_msg", &cal_data);

void setup() {
  ultra.getHardware()->setBaud(57600);
  ultra.initNode();
  ultra.advertise(ultra_data);
//  ultra.subscribe(mod_data);
//  pinMode(5, OUTPUT); //motor Driver-1 Speed Left
//  pinMode(4, OUTPUT); //motor Driver-1 Direction Left
//  pinMode(9, OUTPUT); //motor Driver-2 Speed Right
//  pinMode(8, OUTPUT); //motor Driver-2 Direction Right
  
}

void loop() {

  dis_data.linear.x = hcl.dist();
  dis_data.linear.y = hcc.dist();
  dis_data.linear.z = hcr.dist();
  ultra_data.publish(&dis_data);
  ultra.spinOnce();
  delay(10);
  
}




/*#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <HCSR04.h>

HCSR04 hcl(1, 2);   //Left Sensor
HCSR04 hcc(7, 8);   //Center Sensor
HCSR04 hcr(12, 13); //Right Sensor

ros::NodeHandle  ultra;

geometry_msgs::Twist dis_data;
geometry_msgs::Twist motor_data;
ros::Publisher ultra_data("ultrasound", &dis_data);



void cal_data(const geometry_msgs::Twist& sub_data)
{
  motor_data = sub_data;
  analogWrite(5, motor_data.linear.x);
  analogWrite(6, motor_data.linear.y);
  digitalWrite(3, motor_data.linear.z);
  digitalWrite(4, motor_data.linear.z);  
}

ros::Subscriber<geometry_msgs::Twist> mod_data("mybot", &cal_data);

void setup() {
  
  ultra.initNode();
  ultra.advertise(ultra_data);
  ultra.subscribe(mod_data);
  pinMode(5, OUTPUT); //Motor Driver-1 Speed Left
  pinMode(3, OUTPUT); //Motor Driver-1 Direction Left
  pinMode(6, OUTPUT); //Motor Driver-2 Speed Riight
  pinMode(4, OUTPUT); //Motor Driver-2 Direction Right
  
}

void loop() {

  dis_data.linear.x = hcl.dist();
  dis_data.linear.y = hcc.dist();
  dis_data.linear.z = hcr.dist();
  ultra_data.publish(&dis_data);
  ultra.spinOnce();
  delay(100);
  
}*/