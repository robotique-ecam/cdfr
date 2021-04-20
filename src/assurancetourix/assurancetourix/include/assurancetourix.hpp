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
#include <opencv2/calib3d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
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
  void estimate_arucos_poses();
  void set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type);
  void getTransformation(geometry_msgs::msg::TransformStamped &transformation);
  visualization_msgs::msg::Marker predictEnnemiesPos(visualization_msgs::msg::Marker detectedMarkers);
  void project_corners_pinhole_to_fisheye(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> detected_ids);
  void compute_final_projection(std::vector<std::vector<cv::Point2f>> &coordinates_vector, std::vector<cv::Point2f> &undistort);

#ifdef CAMERA
  // service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
  void handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                     const std_srvs::srv::SetBool::Response::SharedPtr response);
  void init_camera_settings();
  void get_image();
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_aruco_detection;
  VideoCapture _cap;
  struct Camera_settings {
    bool auto_WB;
    int width, height, brightness, contrast, saturation, hue, gamma, gain, backlight_compensation, exposure;
  };
  Camera_settings _camera_settings;
#endif

#ifdef SIMULATION
  int refresh_frequency;
  bool comeback;
  float comeback_x = 1.5;
  std::vector<std::string> robots;
  std::shared_ptr<webots::Supervisor> wb_supervisor;
  void simulation_marker_callback();
#endif

  Mat _frame, _anotated, raised_contrast, tmp;

  std::vector<cv::Vec3d> _rvecs, _tvecs;
  std::vector<int> _small_detected_ids, _huge_detected_ids;
  std::vector<std::vector<cv::Point2f>> _small_marker_corners_projection, _huge_marker_corners_projection;

  double mat_dist_coeffs_fisheye[1][4] = {{-0.036070207475902644, -0.002003213487781793, -0.0007185511458800288, -0.0004833997296696256}};
  double mat_camera_matrix_coeff_fisheye[3][3] = {{1426.4349637104904, 0.0, 1919.8425216336193}, {0.0, 1423.1510790046468, 1094.9067233281635}, {0.0, 0.0, 1.0}};
  float mat_camera_matrix_coeff_fisheye_balanced[3][3] = {{8.59993203e+02, 0.00000000e+00, 1.87111502e+03}, {0.00000000e+00, 8.58013359e+02, 1.16514527e+03}, {0.0, 0.0, 1.0}};

  double mat_dist_coeffs_pinhole[1][5] = {{-0.00131221, -0.00089388, 0.00234124, 0.00322031, 0.00010104}};
  double mat_camera_matrix_coeff_pinhole[3][3] = {{8.51158290e+02, 0.0, 1.89003916e+03}, {0.0, 8.47796721e+02, 1.17912096e+03}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs_fisheye = Mat(4, 1, CV_64F, mat_dist_coeffs_fisheye);
  cv::Mat _cameraMatrix_fisheye = Mat(3, 3, CV_64F, mat_camera_matrix_coeff_fisheye);

  cv::Mat _distCoeffs_pinhole = Mat(5, 1, CV_64F, mat_dist_coeffs_pinhole);
  cv::Mat _cameraMatrix_pinhole = Mat(3, 3, CV_64F, mat_camera_matrix_coeff_pinhole);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

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
  double huge_aruco_size, small_aruco_size;
  int lifetime_sec, lifetime_nano_sec, exposure;
  bool show_image, savedeee;
  uint robot_type, game_element_type;
  std::vector<double> blue_color_ArUco, yellow_color_ArUco, default_color_ArUco, arrow_scale, game_elements_scale;
  std::string base_frame, header_frame_id, topic_for_gradient_layer, side, allies_positions_topic;

  rclcpp::Client<transformix_msgs::srv::TransformixParametersTransformStamped>::SharedPtr transformClient;

#ifdef SIMULATION
  rclcpp::Time get_sim_time(std::shared_ptr<webots::Robot> wb_robot);
#endif /* SIMULATION */
};

#endif /* ONBOARD_VISION_NODE_HPP */
