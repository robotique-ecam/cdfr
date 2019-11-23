#ifndef DRIVE_NODE_HPP
#define DRIVE_NODE_HPP


#include <thread>
#include <math.h>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


#define LEFT 0
#define RIGHT 1


using namespace rclcpp;


class Drive : public rclcpp::Node {
public:
  Drive();
  ~Drive();

private:
  struct TinyCMD {
    /* Speed order for ATTiny going through UART */
    uint8_t left[4] = {0xFF, 1 << 4, 0, 0xFE};
    uint8_t right[4]= {0xFF, 2 << 4, 0, 0xFE};
  };

  struct TinyData {
    /* ATTiny steps from UART */
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

  struct OdometricPose {
    double x = 0;
    double y = 0;
    double thetha = 0;
  };

  // For communicating with ATTiny85 over UART
  std::shared_ptr<serial::Serial> serial_interface_;
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
  int max_freq_;
  int speed_resolution_;

  /* Computed values */
  uint16_t steps_per_turn_;
  double mm_per_turn_;
  double mm_per_step_;
  double speed_multiplier_;
  double max_speed_;
  double min_speed_;

  /* Temporary values */
  double dt = 1; /* us */
  double trajectory_radius = 0;
  double trajectory_radius_left = 0;

  TinyData attiny_speed_cmd_;
  TinyData attiny_steps_returned_;
  TinyCMD differential_speed_cmd_;
  OdometricPose odom_pose_;
  Differential differential_speed_;
  Differential differential_move_;
  Instantaneous instantaneous_speed_;
  Instantaneous instantaneous_move_;

  void init_parameters();
  void init_variables();
  void update_tf();
  void update_joint_states();
  void update_odometry();
  void compute_pose_velocity(TinyData steps_returned);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  uint8_t compute_velocity_cmd(double velocity);

};

#endif /* DRIVE_NODE_HPP */
