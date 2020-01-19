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
#include <cv_bridge/cv_bridge.h>


#define LEFT 0
#define RIGHT 1

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2

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

private:
  VideoCapture _cap;
  Mat _frame, _anotated;
  int _api_id = cv::CAP_ANY;

  std::vector<int> _detected_ids;
  std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  cv_bridge::CvImagePtr cv_img_bridge;
  sensor_msgs::msg::Image img_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // Parameters
  int _camera_id;
};

#endif /* ONBOARD_VISION_NODE_HPP */
