#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String message;
ros::Publisher response_pub("twist_msg_response", &message);

void twist_cb(const geometry_msgs::Twist& msg) {
  String published_str = String("x: " + String(msg.linear.x) + ", ");
  published_str += String("y: " + String(msg.linear.y) + ", ");
  published_str += String("z: " + String(msg.linear.z) + ", ");
  published_str += String("angular x: " + String(msg.angular.x) + ", ");
  published_str += String("angular y: " + String(msg.angular.y) + ", ");
  published_str += String("angular z: " + String(msg.angular.z) + ", ");

  message.data = published_str.c_str();
  response_pub.publish(&message);
}

ros::Subscriber<geometry_msgs::Twist> sub("twist_msg_pub", twist_cb);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(response_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(10);
}
