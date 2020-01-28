#ifndef ONBOARD_VISION_NODE_HPP
#define ONBOARD_VISION_NODE_HPP

#include <math.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>



using namespace rclcpp;
using namespace cv;
using namespace chrono;


class OnboardVision : public rclcpp::Node {
public:
  OnboardVision();
  ~OnboardVision();

private:
  Mat _frame;
  Mat _bwframe;
  Mat _frame_mask;
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
  void detect_hough_circles();
  void find_objects();
  void _mask_circle(cv::Vec3f);
  char _get_dominant_color(Scalar mean, Mat roi, Mat mask);
  Scalar _get_circle_dominant_color(Vec3f detected_circle);
};

#endif /* ONBOARD_VISION_NODE_HPP */
