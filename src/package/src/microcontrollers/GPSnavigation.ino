
#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


ros::NodeHandle  gtg;
geometry_msgs::Twist velocity;
geometry_msgs::Pose goal_pos;
geometry_msgs::Pose curr_pos;

float distance(goal_pos){
  float distance = sqrt(pow(goal_pos.x-curr_pos.x, 2) + pow(goal_pos.y-curr_pos.y));
  return distance;
}

void curr_position(const geometry_msgs::Pose& pos){
  curr_pos = pos;
}

void move_to_pos(){
  Serial.print("Please enter the x-coordinate of your goal: ");
  float goal_pos.x = Serial.parseFloat();
  Serial.print("Please enter the y-coordinate of your goal: ");
  float goal_pos.y = Serial.parseFloat();
  

  while distance(goal_pos) >= 0.01:
  
}


ros::Subscriber<geometry_msgs::Pose> sub("/turtle1/pose", &curr_position)


void setup() {
  gtg.initnode();
  gtg.subscribe(sub);
  move_to_pos(); 

}

void loop() {
  gtg.spinOnce();
  delay(1);
}