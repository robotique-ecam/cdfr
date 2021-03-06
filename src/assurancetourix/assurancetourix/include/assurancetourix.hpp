#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "arducam_mipicamera.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "transformix_msgs/srv/transformix_parameters_transform_stamped.hpp"
#include "transformix_msgs/srv/transformix_parameters_transfrom_pose.hpp"
#ifdef SIMULATION
#include <webots/Supervisor.hpp>
#endif
#include "std_srvs/srv/set_bool.hpp"

using namespace rclcpp;
using namespace cv;
using namespace std::chrono;

class Assurancetourix : public rclcpp::Node {
public:
  Assurancetourix();
  ~Assurancetourix();
  void detect();

private:
  void init_parameters();
  void _detect_aruco(Mat img);
  void _anotate_image(Mat img);
  void set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type);
  void getTransformation(geometry_msgs::msg::TransformStamped &transformation);
  visualization_msgs::msg::Marker predictEnnemiesPos(visualization_msgs::msg::Marker detectedMarkers);

#ifdef MIPI_CAMERA
  // service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
  void handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                     const std_srvs::srv::SetBool::Response::SharedPtr response);
  arducam::CAMERA_INSTANCE camera_instance;
  void get_image();
  int width, height;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_aruco_detection;
#endif

#ifdef EXTERN_CAMERA
  int _api_id = cv::CAP_ANY;
  VideoCapture _cap;
#endif

#ifdef SIMULATION
  int refresh_frequency;
  bool comeback;
  float comeback_x = 1.5;
  std::vector<std::string> robots;
  std::shared_ptr<webots::Supervisor> wb_supervisor;
  void simulation_marker_callback();
#endif

  Mat _frame, _anotated, raised_contrast, image_minus_t, xor_image, xor_image_inv;

  std::vector<int> _detected_ids;
  std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;

  std::vector<cv::Vec3d> _rvecs, _tvecs;

  double mat_dist_coeffs[1][5] = {{0.5 * -0.05507153604545092, -0.036754144220704506, 0.004500420597462287, -0.01843020862512126, 0.017075316951618093}};
  double mat_camera_matrix_coeff[3][3] = {{1.2 * 978.2290511854844, 0.0, 1.2 * 722.0173011119409}, {0.0, 1.2 * 982.4076393453565, 1.2 * 653.1296070005849}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs = Mat(5, 1, CV_64F, mat_dist_coeffs);
  cv::Mat _cameraMatrix = Mat(3, 3, CV_64F, mat_camera_matrix_coeff);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

  visualization_msgs::msg::Marker marker;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  visualization_msgs::msg::Marker transformed_marker;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr transformed_marker_pub_ennemies_, transformed_marker_pub_allies_;

  visualization_msgs::msg::MarkerArray lastEnnemiesMarkers, ennemiesMarkersOnThisCycle;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  geometry_msgs::msg::TransformStamped assurancetourix_to_map_transformation;

  // Parameters
  int _camera_id, mode, lifetime_sec, lifetime_nano_sec, exposure;
  bool show_image, savedeee;
  uint rgain, bgain, robot_type, game_element_type;
  double contrast;
  std::vector<double> blue_color_ArUco, yellow_color_ArUco, default_color_ArUco, arrow_scale, game_elements_scale;
  std::string base_frame, header_frame_id, topic_for_gradient_layer, side, allies_positions_topic;

  rclcpp::Client<transformix_msgs::srv::TransformixParametersTransformStamped>::SharedPtr transformClient;

#ifdef SIMULATION
  rclcpp::Time get_sim_time(std::shared_ptr<webots::Robot> wb_robot);
#endif /* SIMULATION */
};

#endif /* ONBOARD_VISION_NODE_HPP */
