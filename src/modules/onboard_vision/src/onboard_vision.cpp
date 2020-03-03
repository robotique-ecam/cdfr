#include <onboard_vision.hpp>

OnboardVision::OnboardVision() : Node("onboard_vision") {
  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
  }
  exit(-1);
}

void OnboardVision::init_parameters() {
  this->get_parameter_or<int>("camera", _camera_id, 0);
  this->get_parameter_or<bool>("use_roi", _use_roi, false);
  this->get_parameter_or<int>("roi.x_min", _roi_x_min, 0);
  this->get_parameter_or<int>("roi.y_min", _roi_y_min, 0);
  this->get_parameter_or<int>("roi.x_max", _roi_x_max, 1);
  this->get_parameter_or<int>("roi.x_max", _roi_x_max, 1);

  _rect_roi = Rect(_roi_x_min, _roi_y_min, _roi_x_max, _roi_y_max);
}

void OnboardVision::_capture_image() {
  _cap.read(_frame);
  if (_frame.empty())
    RCLCPP_WARN(this->get_logger(), "Grabbed an empty frame");
}

void OnboardVision::_analysis_roi() {
  if (_use_roi)
    _frame = _frame(_rect_roi);
}

Scalar OnboardVision::_get_circle_dominant_color(Vec3f detected_circle) {
  // Get rectangular ROI containing circle
  Rect rect(detected_circle[0], detected_circle[1], detected_circle[2] * 2, detected_circle[2] * 2);
  Mat roi(_frame, rect);
  Mat mask(roi.size(), roi.type(), Scalar::all(0));
  circle(mask, Point(detected_circle[2], detected_circle[2]), detected_circle[2], Scalar::all(255), -1);
}

char OnboardVision::_get_dominant_color(Scalar mean, Mat roi, Mat mask) {}

void OnboardVision::find_objects() {
  // Start by applying a median blur to reduce false positives
  cvtColor(_frame, _bwframe, cv::COLOR_BGR2GRAY);
  medianBlur(_bwframe, _bwframe, 5);

  // Detect some cups
  HoughCircles(_bwframe, _cups_circles,
               cv::HOUGH_GRADIENT, // method
               2,                  // dp
               _bwframe.rows / 4,  // minDist
               200,                // param1
               100,                // param1
               30,                 // min diameter
               70                  // max diameter
  );

  if (_cups_circles.size() > 0)
    for (int i = 0; i < _cups_circles.size(); i++)
      _get_circle_dominant_color(_cups_circles[i]);
}

void OnboardVision::_mask_circle(Vec3f circle) {
  // Get rectangular ROI containing circle
  Rect roi(circle[0], circle[1], circle[2] * 2, circle[2] * 2);
}

OnboardVision::~OnboardVision() { RCLCPP_INFO(this->get_logger(), "Onboard vision node terminated"); }
