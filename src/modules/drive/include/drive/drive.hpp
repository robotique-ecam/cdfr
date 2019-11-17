#ifndef DRIVE_NODE_HPP
#define DRIVE_NODE_HPP


#include <math.h>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


using namespace rclcpp;


class Drive : public rclcpp::Node {
public:
  Drive();
  // ~Drive();

private:
  struct TinyData {
    /* Speed order for ATTiny going through UART */
    int16_t left = 0;
    int16_t right = 0;
  };

  struct Differential {
    /* Computed values of d and theta from steps */
    double left = 0;
    double right = 0;
  };

  struct Instantaneous {
    /* Computed values */
    double linear = 0;
    double angular = 0;
  };

  // For communicating with ATTiny85 over UART
  serial::Serial serial_interface_;
  std::string serial_port_;
  uint32_t serial_baudrate_;

  // ROS time
  rclcpp::Time time_since_last_sync_;
  rclcpp::Time previous_time_since_last_sync_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::TimerBase::SharedPtr timer_base_;

  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;

  double wheel_separation_;
  double wheel_radius_;

  /* Computed values */
  uint16_t steps_per_turn_;
  double mm_per_turn_;
  double mm_per_step_;

  /* Temporary values */
  double dt = 1; /* us */
  double trajectory_radius = 0;
  double trajectory_radius_left = 0;

  TinyData attiny_speed_cmd_;
  TinyData attiny_steps_returned_;
  Differential differential_speed_;
  Differential differential_move_;
  Instantaneous instantaneous_speed_;
  Instantaneous instantaneous_move_;

  void init_parameters();
  void init_variables();
  void steps_returned_callback(TinyData steps_returned);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

};

#endif /* DRIVE_NODE_HPP */
