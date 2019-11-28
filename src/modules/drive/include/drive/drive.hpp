#ifndef DRIVE_NODE_HPP
#define DRIVE_NODE_HPP

#include "i2c.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <chrono>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

#define LEFT 0
#define RIGHT 1

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2

using namespace rclcpp;
using namespace std::chrono;

class Drive : public rclcpp::Node {
public:
  Drive();
  ~Drive();

private:
  struct TinyCMD {
    /* ATTiny steps from UART */
    int8_t left = 0;
    int8_t right = 0;
  };

  struct TinyData {
    /* ATTiny steps from UART */
    int32_t left = 0;
    int32_t right = 0;
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

  template<typename T> void print(std::vector<T> const &v) {
      for (auto i: v) {
          std::cout << "0x" << std::hex << i << ' ';
      }
      std::cout << '\n';
  }

  // For communicating with ATTiny85 over I2C
  std::shared_ptr<I2C> i2c;

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

  /* I2C Bus */
  int i2c_bus;

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

  bool received_steps_left;
  bool received_steps_right;

  TinyData attiny_speed_cmd_;
  TinyData attiny_steps_returned_;
  TinyCMD differential_speed_cmd_;
  OdometricPose odom_pose_;
  Differential differential_speed_;
  Differential differential_move_;
  Instantaneous instantaneous_speed_;
  Instantaneous instantaneous_move_;
  rclcpp::TimerBase::SharedPtr serial_read_timer_;

  void init_parameters();
  void init_variables();
  void update_tf();
  void update_joint_states();
  void update_odometry();
  void read_from_serial();
  void compute_pose_velocity(TinyData steps_returned);
  void steps_received_callback(int32_t steps, uint8_t id);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  int8_t compute_velocity_cmd(double velocity);

};

#endif /* DRIVE_NODE_HPP */
