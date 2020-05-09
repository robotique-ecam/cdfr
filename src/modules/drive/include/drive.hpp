#ifndef DRIVE_NODE_HPP
#define DRIVE_NODE_HPP

#ifndef SIMULATION
#include "i2c.hpp"
#else
#include <limits>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#endif
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <chrono>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

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
    /* ATTiny speed command to I2C */
    int8_t left = 0;
    int8_t right = 0;
  };

  struct TinyData {
    /* ATTiny steps from I2C */
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

#ifndef SIMULATION
  // For communicating with ATTiny85 over I2C
  int i2c_bus;
  std::shared_ptr<I2C> i2c;
#else
  std::shared_ptr<webots::Supervisor> wb_supervisor;
  webots::Motor * wb_left_motor;
  webots::Motor * wb_right_motor;
  webots::PositionSensor * wp_left_encoder;
  webots::PositionSensor * wp_right_encoder;
  Differential old_steps_returned;
  double timestep;
  rclcpp::TimerBase::SharedPtr time_stepper_;
#endif

  // ROS time
  rclcpp::Time time_since_last_sync_;
  rclcpp::Time previous_time_since_last_sync_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

#ifdef USE_TIMER
  rclcpp::TimerBase::SharedPtr timer_;
#endif /* USE_TIMER */

  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;
  diagnostic_msgs::msg::DiagnosticArray diagnostics_array_;
  diagnostic_msgs::msg::DiagnosticStatus diagnostics_status_;

  double accel_;
  double wheel_separation_;
  double wheel_radius_;
  int max_freq_;
  int speed_resolution_;

  /* Computed values */
  uint16_t steps_per_turn_;
  double rads_per_step;
  double meters_per_turn_;
  double meters_per_step_;
  double speed_multiplier_;
  double max_speed_;
  double min_speed_;

  /* Temporary values */
  double dt = 1; /* us */
  double trajectory_radius = 0;
  double trajectory_radius_left = 0;

  bool sign_steps_left = false;
  bool sign_steps_right = false;

  TinyData attiny_speed_cmd_;
  TinyData attiny_steps_returned_;
  TinyCMD differential_speed_cmd_;
  OdometricPose odom_pose_;
  Differential cmd_vel_;
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
  void update_velocity();
  void update_diagnostic();
  void read_from_serial();
  void compute_pose_velocity(TinyData steps_returned);
  void steps_received_callback(int32_t steps, uint8_t id);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  int8_t compute_velocity_cmd(double velocity);
  rclcpp::Time get_sim_time();

  #ifdef SIMULATION
  void sim_step();
  #endif /* SIMULATION */
};

#endif /* DRIVE_NODE_HPP */
