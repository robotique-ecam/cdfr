#ifndef ASSURANCETOURIX_HPP
#define ASSURANCETOURIX_HPP

#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include <chrono>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>
#include <geometrix.hpp>
#ifdef SIMULATION
#include <webots/Supervisor.hpp>
#endif
#include "std_srvs/srv/set_bool.hpp"

using namespace rclcpp;
using namespace cv;
using namespace std::chrono;

class Geometrix;

class Assurancetourix : public rclcpp::Node {
public:
  Assurancetourix();
  ~Assurancetourix();
  visualization_msgs::msg::Marker predict_enemies_pos(visualization_msgs::msg::Marker detectedMarkers);
  std::vector<double> blue_color_aruco, yellow_color_aruco;
  visualization_msgs::msg::MarkerArray last_enemies_markers, enemies_markers_on_this_cycle;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr transformed_marker_pub_ennemies_, transformed_marker_pub_allies_, marker_pub_;
  geometry_msgs::msg::TransformStamped assurancetourix_to_map_transformation;
  std::string side;

private:
  void init_parameters();
  void _detect_aruco(Mat img);
  void estimate_arucos_poses();
  void detection_timer_callback_routine();
  void set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type);
  void project_corners_pinhole_to_fisheye(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> detected_ids);
  void compute_final_projection(std::vector<std::vector<cv::Point2f>> &coordinates_vector, std::vector<cv::Point2f> &undistort);
  void compute_estimation_markers(std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs,
  visualization_msgs::msg::MarkerArray &marker_array_ennemies, visualization_msgs::msg::MarkerArray &marker_array_allies, std::vector<int> detected_ids, visualization_msgs::msg::MarkerArray &markers_camera_relative);

#ifdef CAMERA
  // service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
  void handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                     const std_srvs::srv::SetBool::Response::SharedPtr response);
  void init_camera_settings();
  void get_image();
  void estimate_initial_camera_pose();
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_aruco_detection;
  VideoCapture _cap;
  struct Camera_settings {
    bool auto_WB;
    int width, height, brightness, contrast, saturation, hue, gamma, gain, backlight_compensation, exposure, sharpness;
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

  std::vector<int> _small_detected_ids, _huge_detected_ids, _center_detected_id;
  std::vector<std::vector<cv::Point2f>> _small_marker_corners_projection, _huge_marker_corners_projection, _center_corner_projection;

  double mat_dist_coeffs_fisheye[1][4] = {{-0.04904779188817534, 0.01390908063326121, -0.0115756108661675, 0.0022569907575344978}};
  double mat_camera_matrix_coeff_fisheye[3][3] = {{1445.8697620143487, 0.0, 1920.3261632236809}, {0.0, 1443.631593109402, 1097.1329487765142}, {0.0, 0.0, 1.0}};
  float mat_camera_matrix_coeff_fisheye_balanced[3][3] = {{8.65569129e+02, 0.00000000e+00, 1.95015357e+03}, {0.00000000e+00, 8.64229250e+02, 1.14386597e+03}, {0.0, 0.0, 1.0}};

  double mat_dist_coeffs_pinhole[1][5] = {{0.00697532, -0.00194599, 0.00243645, 0.00061587, 0.00020362}};
  double mat_camera_matrix_coeff_pinhole[3][3] = {{900, 0.0, 1920}, {0.0, 900, 1080}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs_fisheye = Mat(4, 1, CV_64F, mat_dist_coeffs_fisheye);
  cv::Mat _cameraMatrix_fisheye = Mat(3, 3, CV_64F, mat_camera_matrix_coeff_fisheye);

  cv::Mat _distCoeffs_pinhole = Mat(5, 1, CV_64F, mat_dist_coeffs_pinhole);
  cv::Mat _cameraMatrix_pinhole = Mat(3, 3, CV_64F, mat_camera_matrix_coeff_pinhole);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

  visualization_msgs::msg::Marker marker, center_marker;

  visualization_msgs::msg::Marker transformed_marker;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  // Parameters
  double huge_aruco_size, small_aruco_size;
  int lifetime_sec, lifetime_nano_sec, exposure;
  bool show_image, savedeee;
  uint aruco_element_type, game_element_type;
  std::vector<double> prediction_color, default_color_aruco, aruco_element_scale, game_elements_scale, _enemies_arucos_nb;
  std::string base_frame, header_frame_id, topic_for_gradient_layer, allies_positions_topic;

  Geometrix* geometrix;

#ifdef SIMULATION
  rclcpp::Time get_sim_time(std::shared_ptr<webots::Robot> wb_robot);
#endif /* SIMULATION */
};

#endif /* ONBOARD_VISION_NODE_HPP */
