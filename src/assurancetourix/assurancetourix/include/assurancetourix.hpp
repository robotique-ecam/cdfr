#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>


#define LEFT 0
#define RIGHT 1

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2

using namespace rclcpp;
using namespace cv;

class Assurancetourix : public rclcpp::Node {
public:
  Assurancetourix();
  ~Assurancetourix();

private:
  Mat _frame;
  VideoCapture _cap;
  int _api_id = cv::CAP_ANY;

  // Parameters
  int _camera_id;
};

#endif /* ONBOARD_VISION_NODE_HPP */
