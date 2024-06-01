#include <ros.h>
#include <std_msgs/Float32.h>
#include <SoftwareSerial.h>
#include <HCSR04.h>

HCSR04 hc(5, 4);

ros::NodeHandle  ultra;

std_msgs::Float32 dis_data;
std_msgs::Float32 motor_data;
ros::Publisher ultra_data("ultra_msg", &dis_data);

void cal_data(const std_msgs::Float32& data)
{
  motor_data = data;
//  Serial.print("Modified Data: " + String(&motor_data);
//  delay(60);
//  analogWrite(3, int(motor_data);
}

ros::Subscriber<std_msgs::Float32> mod_data("mod_msg", &cal_data);

void setup() {
  // put your setup code here, to run once:
  // pinMode(3, INPUT);
  ultra.initNode();
  ultra.advertise(ultra_data);
  ultra.subscribe(mod_data);
//  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  dis_data.data = hc.dist();
  ultra_data.publish(&dis_data);
//  Serial.println("Distance: " + String(hc.dist()));
//  delay(60); 
  ultra.spinOnce();
  delay(1000);
}