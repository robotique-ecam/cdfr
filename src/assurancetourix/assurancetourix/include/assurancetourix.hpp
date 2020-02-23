#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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

  #ifdef MIPI_CAMERA
  arducam::CAMERA_INSTANCE camera_instance;
  int width = 1920, height = 1080;
  void get_image();
  #else
  int _api_id = cv::CAP_ANY;
  VideoCapture _cap;
  #endif // MIPI_CAMERA

  Mat _frame, _anotated;

  std::vector<int> _detected_ids;
  std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;

  std::vector<cv::Vec3d> _rvecs, _tvecs;

  /* TODO: establish the new coeffss with the camera */
  double mat_dist_coeffs[1][5] = {{0.3764196381675952, 0.04110048800291252, 0.22561542466361806, -0.04945838920708183, -0.4677971323451955}};
  double mat_camera_matrix_coeff[3][3] = {{3667.6295530241346, 0.0, 1309.653212735876}, {0.0, 3009.8721997418875, 1565.0383212417412}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs = Mat(5,1, CV_64F, mat_dist_coeffs);
  cv::Mat _cameraMatrix = Mat(3,3, CV_64F, mat_camera_matrix_coeff);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  cv_bridge::CvImage cv_img_bridge;
  sensor_msgs::msg::Image img_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;

  visualization_msgs::msg::Marker marker;
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  //useless point and colors, just to fill up the vector of points for the pointer
  geometry_msgs::msg::Point useless_point;
  std::vector<geometry_msgs::msg::Point> useless_point_vector;

  std::vector<std_msgs::msg::ColorRGBA> useless_color_vector;

  std_msgs::msg::ColorRGBA blue_color_ArUco;
  std_msgs::msg::ColorRGBA yellow_color_ArUco;
  std_msgs::msg::ColorRGBA default_color_ArUco;
  // Parameters
  int _camera_id;
};

#endif /* ONBOARD_VISION_NODE_HPP */
