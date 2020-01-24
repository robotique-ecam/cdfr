#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  _cap.open(_api_id + _camera_id);
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device : %d : API %d", _camera_id, _api_id);
    exit(-1);
  }

  //auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  image_pub_ = image_transport::create_publisher(this, "detected_aruco", rmw_qos_profile_default);

  timer_ = this->create_wall_timer(1s, std::bind(&Assurancetourix::detect, this));

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");

}


void Assurancetourix::init_parameters() {
  cv_img_bridge.encoding = "bgr8";
  cv_img_bridge.header.frame_id = "robotbase";
}


void Assurancetourix::detect() {

  _cap.read(_frame);
  _frame.copyTo(_anotated);
  cv_img_bridge.header.stamp = this->get_clock()->now();

  _detect_aruco(_anotated);
  _anotate_image(_anotated);

  cv_img_bridge.image = _anotated;
  cv_img_bridge.toImageMsg(img_msg);

  //pos_rot_ids = this->create_publisher<assurancetourix_msg::msg::AssurancetourixMsg>("vision_position", qos);
  //image_pub_.publish(img_msg);

}


void Assurancetourix::_detect_aruco(Mat img) {
  cv::aruco::detectMarkers(img, _dictionary, _marker_corners, _detected_ids, _parameters, _rejected_candidates);
}


void Assurancetourix::_anotate_image(Mat img) {
  //std_msgs::msg::int[] _detected_ids_list;
  if (_detected_ids.size() > 0)
  {
    cv::aruco::drawDetectedMarkers(img, _marker_corners, _detected_ids);
    //RCLCPP_INFO(this->get_logger(), "number of ArUco detected: %d", _detected_ids.size());
    cv::aruco::estimatePoseSingleMarkers(_marker_corners, 0.05, _cameraMatrix, _distCoeffs, _rvecs, _tvecs);
    //RCLCPP_INFO(this->get_logger(), "id: %d, %f, %f, %f, %f",  _detected_ids.size(),_rvecs[0].operator[](0), _rvecs[0].operator[](1), _rvecs[0].operator[](2), _rvecs[0].operator[](1));

    //int* _detected_ids_list = &_detected_ids[0];

    //pos_rot_ids.ids = _detected_ids_list;

    if (_mat_pos_rot.size() >= 1) {
      _mat_pos_rot.clear();
    }

    for(int i=0; i<int(_detected_ids.size()); i++) {

      cv::aruco::drawAxis(img, _cameraMatrix, _distCoeffs, _rvecs[i], _tvecs[i], 0.1);

      pose.position.x = float(_tvecs[i].operator[](0));
      pose.position.y = float(_tvecs[i].operator[](1));
      pose.position.z = float(_tvecs[i].operator[](2));
      pose.orientation.x = float(_rvecs[i].operator[](0));
      pose.orientation.y = float(_rvecs[i].operator[](1));
      pose.orientation.z = float(_rvecs[i].operator[](2));
      pose.orientation.w = 0;
      _mat_pos_rot.push_back(pose);

      //_detected_ids_list[i] = _detected_ids[i];
      //RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", float(_tvecs[i].operator[](0)), float(_tvecs[i].operator[](1)), float(_tvecs[i].operator[](2)), float(_rvecs[i].operator[](1)));
      RCLCPP_INFO(this->get_logger(), "id: %d", _detected_ids[i]);
    }

    pos_rot_ids.position = _mat_pos_rot;
    //pos_rot_ids.ids = _detected_ids_list;


    //cv::imshow("out.png", img);
  }
}


Assurancetourix::~Assurancetourix() {
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
