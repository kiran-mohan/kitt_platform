/**@file kitt_teensy_ros_node.ino */
/*
 * Kitt Teensy ROS Node
 * 
 * This code is the ROS node that executes on the Teensy processor
 * for the Kitt Car team.
 * 
 * It uses the rosserial library to communicate with ROS. It uses
 * Arduino and Teensy libraries to interact with systems and
 * circuits connected to the Teensy processor. This code assumes
 * that a Teensy 3.5 is used.
 * 
 * https://github.com/kittcar
 * 
 */

// ROS includes
#include <ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

ros::NodeHandle ros_nh;

std_msgs::Header heartbeat_msg;
ros::Publisher heartbeat_pub("heartbeat", &heartbeat_msg);

std_msgs::Int32 param_msg;
ros::Publisher param_pub("param", &param_msg);

void listenerCallback(const std_msgs::Empty& listener_msg);
ros::Subscriber<std_msgs::Empty> listener_sub("listener", listenerCallback);

const int LED_PIN = 13;

/**
 * @brief toggle LED state
 * @param std_msgs::Empty an empty message
 */
void listenerCallback(const std_msgs::Empty& listener_msg)
{
  uint8_t toggled_led_state;
  if (digitalRead(LED_PIN) == HIGH)
  {
    toggled_led_state = LOW;
  }
  else
  {
    toggled_led_state = HIGH;
  }
  digitalWrite(LED_PIN, toggled_led_state);
}

void setup()
{
  ros_nh.advertise(heartbeat_pub);
  ros_nh.advertise(param_pub);
  ros_nh.subscribe(listener_sub);
  ros_nh.getHardware()->setBaud(115200);
  ros_nh.initNode();
  while (!ros_nh.connected())
  {
    ros_nh.spinOnce();
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  int tmp = 0;
  if (ros_nh.getParam("tmp_param", &tmp))
  {
    param_msg.data = tmp;
  }
  else
  {
    param_msg.data = 9999;
  }
}

void loop()
{
  // Publish heartbeat_msg every 1 second
  heartbeat_msg.stamp = ros_nh.now();
  heartbeat_pub.publish(&heartbeat_msg);
  param_pub.publish(&param_msg);
  ros_nh.spinOnce();
  delay(1000);
}

