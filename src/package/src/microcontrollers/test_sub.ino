
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>
ros::NodeHandle Listener;

void messageserial(const std_msgs::String &serial_msg){
  Serial.print(serial_msg);
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageserial);


void setup() {
  Listener.initNode();
  Listener.subscribe(sub);

}

void loop() {
  Listener.spinOnce;
  delay(1);
}