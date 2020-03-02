#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <vector>
#include <chrono>
#include <bits/stdc++.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/video.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "arducam_mipicamera.hpp"

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

  #ifdef MIPI_CAMERA
  arducam::CAMERA_INSTANCE camera_instance;
  void get_image();
  #else
  int _api_id = cv::CAP_ANY;
  VideoCapture _cap;
  #endif // MIPI_CAMERA

  Mat _frame, _anotated, raised_contrast, image_minus_t, xor_image, xor_image_inv;

  std::vector<int> _detected_ids;
  std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;

  std::vector<cv::Vec3d> _rvecs, _tvecs;

  double mat_dist_coeffs[1][5] = {{-0.2516882849093723, 0.16911322215886648, -0.026507870194754653, -0.015272988672555004, -0.07677243583649554}};
  double mat_camera_matrix_coeff[3][3] = {{1386.7837474992405, 0.0, 625.7871833868004}, {0.0, 2292.0151491679253, 838.987898863883}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs = Mat(5,1, CV_64F, mat_dist_coeffs);
  cv::Mat _cameraMatrix = Mat(3,3, CV_64F, mat_camera_matrix_coeff);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  rclcpp::TimerBase::SharedPtr timer_;

  visualization_msgs::msg::Marker marker;
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  int _camera_id;

  int width;
  int height;
  bool show_image;
  int exposure;
  uint rgain;
  uint bgain;
  int mode;
  double contrast;
  std::vector<double> blue_color_ArUco;
  std::vector<double> yellow_color_ArUco;
  std::vector<double> default_color_ArUco;
  std::vector<double> arrow_scale;
  std::vector<double> game_elements_scale;
  uint robot_type;
  uint game_element_type;
  int lifetime_sec;
  int lifetime_nano_sec;
  String header_frame_id;
  bool starter_flag;

};

#endif /* ONBOARD_VISION_NODE_HPP */
