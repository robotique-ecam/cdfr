#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//#include <std_msgs/msg/string.h> see if the conversion with frame.data works
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "transformix_msgs/srv/transformix_parameters.hpp"
#include "transformix_msgs/srv/transformix_parameters_transform_stamped.hpp"
#include "transformix_msgs/srv/transformix_parameters_transfrom_pose.hpp"

using namespace rclcpp;
using namespace std::chrono;

class Transformix : public rclcpp::Node {
public:
  Transformix();
  ~Transformix();

  static void generate_buffer_and_transform_my_pose_stamped(const std::shared_ptr<transformix_msgs::srv::TransformixParameters::Request> request,
                                                            std::shared_ptr<transformix_msgs::srv::TransformixParameters::Response> response);
  static void get_transform_stamped_from_frames(const std::shared_ptr<transformix_msgs::srv::TransformixParametersTransformStamped::Request> request,
                                                std::shared_ptr<transformix_msgs::srv::TransformixParametersTransformStamped::Response> response);
  static void get_poseStamped_from_transformStamped(const std::shared_ptr<transformix_msgs::srv::TransformixParametersTransfromPose::Request> request,
                                                    std::shared_ptr<transformix_msgs::srv::TransformixParametersTransfromPose::Response> response);

private:
  static void get_transform(tf2_ros::Buffer &tfBuffer, std::string &from_frame, std::string &to_frame, geometry_msgs::msg::TransformStamped &transformStamped);
  static bool canTransform(tf2_ros::Buffer &tfBuffer, std::string &from_frame, std::string &to_frame);
  static void setPose(geometry_msgs::msg::PoseStamped &pose, std::string frame, double posx, double posy, double posz, double rotw, double rotx, double roty, double rotz);
  static void setTransform(geometry_msgs::msg::TransformStamped &transform, std::string frame_id, std::string child_frame_id, double posx, double posy, double posz, double rotw,
                           double rotx, double roty, double rotz);

  rclcpp::Service<transformix_msgs::srv::TransformixParameters>::SharedPtr service_all;
  rclcpp::Service<transformix_msgs::srv::TransformixParametersTransformStamped>::SharedPtr service_transform;
  rclcpp::Service<transformix_msgs::srv::TransformixParametersTransfromPose>::SharedPtr service_pose;
};
