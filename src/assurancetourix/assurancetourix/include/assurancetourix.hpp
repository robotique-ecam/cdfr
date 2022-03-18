#ifndef ASSURANCETOURIX_HPP
#define ASSURANCETOURIX_HPP

#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include <chrono>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifdef CAMERA
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>
#include <geometrix.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <transformix_msgs/srv/initial_static_t_fsrv.hpp>
#endif //CAMERA

#ifdef SIMULATION
#include <webots/Supervisor.hpp>
#endif //SIMULATION


using namespace rclcpp;
using namespace std::chrono;

class Geometrix;

#ifdef CAMERA
struct SideSelectionTransfer {
  rclcpp::Client<transformix_msgs::srv::InitialStaticTFsrv>::SharedPtr initial_tf_client;
  rclcpp::Client<transformix_msgs::srv::InitialStaticTFsrv>::SharedRequest request_initial_tf;
  rclcpp::Client<transformix_msgs::srv::InitialStaticTFsrv>::SharedFuture future_initial_tf;
  bool spinning_request;
  rclcpp::TimerBase::SharedPtr timer;
  std::string service_name;
};
#endif //CAMERA

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

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

  uint aruco_element_type;
  std::vector<double> prediction_color;
  std::string topic_for_gradient_layer, allies_positions_topic;

#ifdef CAMERA
  // service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
  void handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                     const std_srvs::srv::SetBool::Response::SharedPtr response);
  void init_camera_settings();
  void get_image();
  void estimate_initial_camera_pose();
  void _detect_aruco(cv::Mat img);
  void estimate_arucos_poses();
  void detection_timer_callback_routine();
  void set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type);
  void project_corners_pinhole_to_fisheye(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> detected_ids);
  void compute_final_projection(std::vector<std::vector<cv::Point2f>> &coordinates_vector, std::vector<cv::Point2f> &undistort);
  void compute_estimation_markers(std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs,
  visualization_msgs::msg::MarkerArray &marker_array_ennemies, visualization_msgs::msg::MarkerArray &marker_array_allies, std::vector<int> detected_ids, visualization_msgs::msg::MarkerArray &markers_camera_relative);
  void get_color_from_position(geometry_msgs::msg::Point &position, std_msgs::msg::ColorRGBA &color, int square_to_check = 7);
  cv::Point2d get_pixels_from_position(geometry_msgs::msg::Point &position);
  void reef_goblet_callback();
  int is_goblet_at_position(geometry_msgs::msg::Point &position);
  void compass_orientation_callback();
  void set_auto_exposure();
  void init_side_selection_st(SideSelectionTransfer &st_side_selection, std::string  service_name);
  void timer_side_client_callback(SideSelectionTransfer &st_side_selection);

  SideSelectionTransfer side_selection_asterix_localisation, side_selection_obelix_localisation;
  SideSelectionTransfer side_selection_asterix_cetautomatix, side_selection_obelix_cetautomatix;
  SideSelectionTransfer side_selection_strategix;

  cv::VideoCapture _cap;
  struct Camera_settings {
    bool auto_WB;
    int width, height, brightness, contrast, saturation, hue, gamma, gain, backlight_compensation, exposure, sharpness;
  };
  Camera_settings _camera_settings;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_aruco_detection;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compass_pub;

  rclcpp::TimerBase::SharedPtr timer_compass_;

  cv::Mat _frame, _anotated, raised_contrast, tmp;

  std::vector<int> _small_detected_ids, _huge_detected_ids, _center_detected_id;
  std::vector<std::vector<cv::Point2f>> _small_marker_corners_projection, _huge_marker_corners_projection, _center_corner_projection;

  double mat_dist_coeffs_fisheye[1][4] = {{-0.04904779188817534, 0.01390908063326121, -0.0115756108661675, 0.0022569907575344978}};
  double mat_camera_matrix_coeff_fisheye[3][3] = {{1445.8697620143487, 0.0, 1920.3261632236809}, {0.0, 1443.631593109402, 1097.1329487765142}, {0.0, 0.0, 1.0}};
  float mat_camera_matrix_coeff_fisheye_balanced[3][3] = {{8.65569129e+02, 0.00000000e+00, 1.95015357e+03}, {0.00000000e+00, 8.64229250e+02, 1.14386597e+03}, {0.0, 0.0, 1.0}};

  double mat_dist_coeffs_pinhole[1][5] = {{0.00697532, -0.00194599, 0.00243645, 0.00061587, 0.00020362}};
  double mat_camera_matrix_coeff_pinhole[3][3] = {{900, 0.0, 1920}, {0.0, 900, 1080}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs_fisheye = cv::Mat(4, 1, CV_64F, mat_dist_coeffs_fisheye);
  cv::Mat _cameraMatrix_fisheye = cv::Mat(3, 3, CV_64F, mat_camera_matrix_coeff_fisheye);

  cv::Mat _distCoeffs_pinhole = cv::Mat(5, 1, CV_64F, mat_dist_coeffs_pinhole);
  cv::Mat _cameraMatrix_pinhole = cv::Mat(3, 3, CV_64F, mat_camera_matrix_coeff_pinhole);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

  geometry_msgs::msg::TransformStamped assurancetourix_to_map_tf_inv;

  visualization_msgs::msg::Marker marker, center_marker, transformed_marker;

  Geometrix* geometrix;

  // Parameters
  double huge_aruco_size, small_aruco_size;
  int lifetime_sec, lifetime_nano_sec, exposure;
  bool show_image, first_image_saved;
  uint game_element_type;
  std::vector<double> default_color_aruco, aruco_element_scale, game_elements_scale, _enemies_arucos_nb;
  std::string base_frame, header_frame_id;
#endif

#ifdef SIMULATION
  rclcpp::Time get_sim_time(std::shared_ptr<webots::Robot> wb_robot);

  int refresh_frequency;
  bool comeback;
  float comeback_x = 1.5;
  std::vector<std::string> robots;
  std::shared_ptr<webots::Supervisor> wb_supervisor;
  void simulation_marker_callback();
#endif //SIMULATION
};

#endif /* ONBOARD_VISION_NODE_HPP */
