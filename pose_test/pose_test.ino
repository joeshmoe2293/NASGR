/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;


bool set_; 


geometry_msgs::Pose sum_msg;
ros::Publisher p("sum", &sum_msg);

std_msgs::String response;
ros::Publisher str_pub("sum_messages", &response);

void messageCb(const geometry_msgs::Pose& msg){
  sum_msg.position.x = msg.position.x + 1;
  sum_msg.position.y = msg.position.y + 1;
  sum_msg.position.z = msg.position.z + 1;
  sum_msg.orientation.x = msg.orientation.x;
  sum_msg.orientation.y = msg.orientation.y;
  sum_msg.orientation.z = msg.orientation.z;
  sum_msg.orientation.w = msg.orientation.w;
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  response.data = "Publishing sum";
  str_pub.publish(&response);
}

ros::Subscriber<geometry_msgs::Pose> s("poses", messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  nh.advertise(str_pub);
}

void loop()
{  
  p.publish(&sum_msg);
  nh.spinOnce();
  delay(10);
}

