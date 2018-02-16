/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

volatile bool toggleBit = false;

void messageCb( const std_msgs::Int8& toggle_msg){
  if (toggle_msg.data > 0) {
    toggleBit = !toggleBit;
    auto value = (toggleBit) ? HIGH:LOW;
    digitalWrite(13, value);   // blink the led
  }
}

ros::Subscriber<std_msgs::Int8> sub("toggle_led", messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
