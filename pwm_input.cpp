#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

void setDutyCycle(int dutyCycle) {
  std::ofstream pwmFile;
  pwmFile.open("/sys/class/pwm/pwmchip4/pwm1/duty_cycle");
  pwmFile << dutyCycle;
  pwmFile.close();
}

void flightCommandCallback(const std_msgs::Float32::ConstPtr& msg) {
  // Extract the desired duty cycle from the message
  float dutyCycle = msg->data;

  // Update the duty cycle
  setDutyCycle(dutyCycle);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to the MAVROS topic for flight commands
  ros::Subscriber sub = nh.subscribe("flight_commands", 1, flightCommandCallback);

  // Set the duty cycle to 1400000
  setDutyCycle(1400000);

  // Wait for some time before updating the duty cycle
  ros::Duration(5).sleep();

  // Update the duty cycle to 800000
  setDutyCycle(800000);

  // Wait for some time before updating the duty cycle
  ros::Duration(5).sleep();

  // Update the duty cycle to 1200000
  setDutyCycle(1200000);

  ros::spin();

  return 0;
}
