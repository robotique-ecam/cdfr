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
#include <tf2/LinearMath/Quaternion.h>


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

  /* when camera calibration will be done */
  std::vector<cv::Vec3d> _rvecs, _tvecs;
  std::vector<tf2::Quaternion> _mat_pos_rot;

  /* TODO: establish the new coeffss with the camera */
  double mat_dist_coeffs[1][5] = {{0.3500038366337939, -1.4933155679576624, 0.022462074105878548, -0.008107582986986875, 2.9089180661290976}};
  double mat_camera_matrix_coeff[3][3] = {{500.1787284360219, 0.0, 306.89160844028396}, {0.0, 503.06670218899563, 248.89861392261338}, {0.0, 0.0, 1.0}};

  cv::Mat _distCoeffs = Mat(5,1, CV_64F, mat_dist_coeffs);
  cv::Mat _cameraMatrix = Mat(3,3, CV_64F, mat_camera_matrix_coeff);

  cv::Ptr<cv::aruco::DetectorParameters> _parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  cv_bridge::CvImage cv_img_bridge;
  sensor_msgs::msg::Image img_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher image_pub_;

  // Parameters
  int _camera_id;
};

#endif /* ONBOARD_VISION_NODE_HPP */
