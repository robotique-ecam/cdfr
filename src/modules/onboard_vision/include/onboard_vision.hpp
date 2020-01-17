#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>



#define LEFT 0
#define RIGHT 1

#define STEPPER_LEFT 1
#define STEPPER_RIGHT 2

using namespace rclcpp;
using namespace cv;

class OnboardVision : public rclcpp::Node {
public:
  OnboardVision();
  ~OnboardVision();

private:
  Mat _frame;
  Mat _bwframe;
  Rect _rect_roi;
  VideoCapture _cap;
  std::vector<Vec3f> _cups_circles;

  int _api_id = cv::CAP_ANY;

  // Parameters
  int _camera_id;
  bool _use_roi;
  int _roi_x_min;
  int _roi_y_min;
  int _roi_x_max;
  int _roi_y_max;


  void init_parameters();
  void _capture_image();
  void _analysis_roi();
  void _undistort_images();
  void _detect_hough_circles();
  void _mask_circle(Vec3f circle);
  void _get_dominant_color();
  void find_objects();
};

#endif /* ONBOARD_VISION_NODE_HPP */
