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
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDrive.h>

// C++ includes
#include <math.h>

ros::NodeHandle ros_nh;

std_msgs::Header heartbeat_msg;
ros::Publisher heartbeat_pub("heartbeat", &heartbeat_msg);

std_msgs::String error_msg;
ros::Publisher error_pub("platform/debug/error", &error_msg);

ackermann_msgs::AckermannDrive debug_drive_msg;
ros::Publisher debug_drive_pub("platform/debug/drive", &debug_drive_msg);

void listenerCallback(const std_msgs::Empty& listener_msg);
ros::Subscriber<std_msgs::Empty> listener_sub("listener", listenerCallback);

void driveCallback(const ackermann_msgs::AckermannDrive& drive_msg);
ros::Subscriber<ackermann_msgs::AckermannDrive> drive_sub("platform/drive", driveCallback);

const int LED_PIN = 13;

// ROS params
// steering offset in radians
float steering_offset = M_PI / 2;
// min_speed in m/s
float min_speed = 0.0;
// max_speed in m/s
float max_speed = 5.0;

bool params_retrieved_successfully = true;

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

/**
 * @brief echo received message onto /platform/debug/drive
 * @param ackermann_msgs::AckermannDrive the received drive message
 */
void driveCallback(const ackermann_msgs::AckermannDrive& drive_msg)
{
  debug_drive_msg = drive_msg;
  debug_drive_pub.publish(&debug_drive_msg);
}

/**
 * @brief get all the required params from the ROS param server
 */
bool getRequiredParams()
{
  if (!ros_nh.getParam("drive/steering_offset", &steering_offset))
  {
    return false;
  }
  if(!ros_nh.getParam("drive/min_speed", &min_speed))
  {
    return false;
  }
  if(!ros_nh.getParam("drive/max_speed", &max_speed))
  {
    return false;
  }
  return true;
}

void setup()
{
  ros_nh.advertise(heartbeat_pub);
  ros_nh.advertise(debug_drive_pub);
  ros_nh.advertise(error_pub);
  ros_nh.subscribe(listener_sub);
  ros_nh.subscribe(drive_sub);
  ros_nh.getHardware()->setBaud(115200);
  ros_nh.initNode();
  while (!ros_nh.connected())
  {
    ros_nh.spinOnce();
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  params_retrieved_successfully = getRequiredParams();
}

void loop()
{
  // Publish heartbeat_msg every 1 second
  heartbeat_msg.stamp = ros_nh.now();
  heartbeat_pub.publish(&heartbeat_msg);
  if (!params_retrieved_successfully)
  {
    error_msg.data = "Error retrieving params";
    error_pub.publish(&error_msg);
  }
  ros_nh.spinOnce();
  delay(1000);
}
