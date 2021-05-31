#ifndef DRIVE_NODE_HPP
#define DRIVE_NODE_HPP

#ifndef SIMULATION

#include <ODrive.hpp>
#include "i2c.hpp"

#else

#include <limits>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>

#endif //SIMULATION

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "actuators_srvs/srv/slider.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <chrono>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#define LEFT 0
#define RIGHT 1

using namespace rclcpp;
using namespace std::chrono;

class Drive : public rclcpp::Node {
public:
  Drive();
  ~Drive();

private:
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
  std::string _odrive_usb_port;
  bool _invert_wheel;
  int odrive_motor_order[2] = {0, 1};
  double _gearbox_ratio;
  Differential old_motor_turns_returned;
  ODrive *odrive;
  void get_motors_turns_from_odrive(double &left, double &right);
  float compute_velocity_cmd(double velocity);
  // For communicating with the slider over I2C
  int i2c_bus;
  std::shared_ptr<I2C> i2c;
  std::mutex i2c_mutex;
#else
  std::shared_ptr<webots::Robot> wb_robot;
  webots::Motor
    *wb_left_motor, *wb_right_motor;
  webots::PositionSensor
    *wp_left_encoder, *wp_right_encoder;
  Differential old_turns_returned;
  double timestep;
  rclcpp::TimerBase::SharedPtr time_stepper_;
  void sim_step();
  rclcpp::Time get_sim_time();
#endif //SIMULATION

  // ROS time
  rclcpp::Time
    time_since_last_sync_, previous_time_since_last_sync_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Steppers disable service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_drivers;

  // Sliders service
  rclcpp::Service<actuators_srvs::srv::Slider>::SharedPtr _set_slider_postion;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

#ifdef USE_TIMER
  rclcpp::TimerBase::SharedPtr timer_;
#endif /* USE_TIMER */

  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;
  diagnostic_msgs::msg::DiagnosticArray diagnostics_array_;
  diagnostic_msgs::msg::DiagnosticStatus diagnostics_status_;

  /* Odometry re-adjustement */
  geometry_msgs::msg::TransformStamped _map_to_odom_tf;
  std::vector<geometry_msgs::msg::TransformStamped> _previous_tf;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr _adjust_odometry_sub;

  double _wheel_separation, _wheel_radius;

  /* Computed values */
  double _wheel_circumference, dt; /* us */

  Differential
    wheels_turns_returned_, cmd_vel_, differential_speed_, differential_move_;

  Instantaneous
    instantaneous_speed_, instantaneous_move_;

  OdometricPose odom_pose_;

  void init_parameters();
  void init_variables();
  void update_tf();
  void update_joint_states();
  void update_odometry();
  void update_velocity();
  void update_diagnostic();
  void compute_pose_velocity(Differential turns_returned);
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void handle_drivers_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                             const std_srvs::srv::SetBool::Response::SharedPtr response);
  void handle_set_slider_position(const std::shared_ptr<rmw_request_id_t> request_header, const actuators_srvs::srv::Slider::Request::SharedPtr request,
                                  const actuators_srvs::srv::Slider::Response::SharedPtr response);
  void adjust_odometry_callback(const geometry_msgs::msg::TransformStamped::SharedPtr tf_stamped_msg);
  void extract_pose_from_transform(geometry_msgs::msg::TransformStamped &transform_in, geometry_msgs::msg::PoseStamped &pose_out);
  void set_transform_from_pose(geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::TransformStamped &transform_out);
  void get_pose_in_another_frame(geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::PoseStamped &pose_out, geometry_msgs::msg::TransformStamped &transform);
  tf2::Quaternion get_tf2_quaternion(geometry_msgs::msg::Quaternion &q);
  tf2::Vector3 get_tf2_vector(geometry_msgs::msg::Point &p);
  tf2::Vector3 get_tf2_vector(geometry_msgs::msg::Vector3 &p);
  void set_pose_from_tf_t_q(tf2::Vector3 &t, tf2::Quaternion &q, geometry_msgs::msg::PoseStamped &pose_out);
  double dummy_tree_digits_precision(double a);
};

#endif /* DRIVE_NODE_HPP */
