#include <assurancetourix.hpp>

Assurancetourix::Assurancetourix() : Node("assurancetourix") {
  /* Init parametrers from YAML */
  init_parameters();

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_aruco_position", qos);
  transformed_marker_pub_ennemies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_for_gradient_layer, qos);
  transformed_marker_pub_allies_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(allies_positions_topic, qos);

#ifdef SIMULATION
  RCLCPP_INFO(this->get_logger(), "Starting SIMULATION mode");
  RCLCPP_INFO(this->get_logger(), "Simulated robots:");
  for (auto robot : robots) {
    RCLCPP_INFO(this->get_logger(), "%s", robot.c_str());
  }

  char mypath[] = "WEBOTS_ROBOT_NAME=game_manager";
  putenv(mypath);

  wb_supervisor = std::make_shared<webots::Supervisor>();
  comeback = false;

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / refresh_frequency), std::bind(&Assurancetourix::simulation_marker_callback, this));
#endif

#ifdef CAMERA

  RCLCPP_INFO(this->get_logger(), "Starting CAMERA mode");

  init_camera_settings();

  timer_ = NULL;

  compass_pub = this->create_publisher<std_msgs::msg::String>("/compass", qos);
  _enable_aruco_detection = this->create_service<std_srvs::srv::SetBool>(
      "/enable_aruco_detection", std::bind(&Assurancetourix::handle_aruco_detection_enable, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  _parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
  _parameters->cornerRefinementWinSize = 11;

  geometrix = new Geometrix(this);
  estimate_initial_camera_pose();

  RCLCPP_WARN(this->get_logger(), "Assurancetourix camera position guessing x: %f y: %f", assurancetourix_to_map_transformation.transform.translation.x,
    assurancetourix_to_map_transformation.transform.translation.y);

  if (assurancetourix_to_map_transformation.transform.translation.x < 1.5) this->declare_parameter<std::string>("side", "blue");
  else this->declare_parameter<std::string>("side", "yellow");
  this->get_parameter("side", side);

  set_auto_exposure();

  timer_compass_ = this->create_wall_timer(std::chrono::seconds(15), std::bind(&Assurancetourix::compass_orientation_callback, this));

  if (show_image) {
    cv::namedWindow("anotated", cv::WINDOW_AUTOSIZE);
  }
  first_image_saved = false;

#endif // CAMERA

  RCLCPP_INFO(this->get_logger(), "Assurancetourix has been started");
}

void Assurancetourix::init_parameters() {

  // declare variables from yml
  this->declare_parameter("topic_for_gradient_layer");
  this->declare_parameter("topic_for_allies_position");
  this->declare_parameter("rviz_settings.prediction_color");
  this->declare_parameter("rviz_settings.aruco_element_type");

  this->get_parameter_or<std::string>("topic_for_gradient_layer", topic_for_gradient_layer, "/default_topic_for_gradient_layer");
  this->get_parameter_or<std::string>("topic_for_allies_position", allies_positions_topic, "/default_topic_for_allies_positions");
  this->get_parameter_or<std::vector<double>>("rviz_settings.prediction_color", prediction_color, {185.0, 76.0, 225.0, 0.5});
  this->get_parameter_or<uint>("rviz_settings.aruco_element_type", aruco_element_type, 2);

#ifdef CAMERA
  this->declare_parameter("image_post_processing.show_image");
  this->declare_parameter("arucos.small_aruco_size");
  this->declare_parameter("arucos.huge_aruco_size");

  this->get_parameter_or<bool>("image_post_processing.show_image", show_image, false);
  this->get_parameter_or<double>("arucos.small_aruco_size", small_aruco_size, 0.07);
  this->get_parameter_or<double>("arucos.huge_aruco_size", huge_aruco_size, 0.115);

  this->declare_parameter("rviz_settings.blue_color_aruco");
  this->declare_parameter("rviz_settings.yellow_color_aruco");
  this->declare_parameter("rviz_settings.default_color_aruco");
  this->declare_parameter("rviz_settings.aruco_element_scale");
  this->declare_parameter("rviz_settings.game_elements_scale");
  this->declare_parameter("rviz_settings.game_element_type");
  this->declare_parameter("rviz_settings.lifetime_sec");
  this->declare_parameter("rviz_settings.header_frame_id");

  this->get_parameter_or<std::vector<double>>("rviz_settings.blue_color_aruco", blue_color_aruco, {0.0, 0.0, 255.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.yellow_color_aruco", yellow_color_aruco, {255.0, 255.0, 0.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.default_color_aruco", default_color_aruco, {120.0, 120.0, 120.0});
  this->get_parameter_or<std::vector<double>>("rviz_settings.aruco_element_scale", aruco_element_scale, {0.05, 0.05, 0.05});
  this->get_parameter_or<std::vector<double>>("rviz_settings.game_elements_scale", game_elements_scale, {0.07, 0.07, 0.01});
  this->get_parameter_or<uint>("rviz_settings.game_element_type", game_element_type, 1);
  this->get_parameter_or<int>("rviz_settings.lifetime_sec", lifetime_sec, 2);
  this->get_parameter_or<std::string>("rviz_settings.header_frame_id", header_frame_id, "assurancetourix");

  // initialisation of markers
  marker.header.frame_id = header_frame_id;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime.sec = lifetime_sec;

  base_frame = "map";
  transformed_marker.header.frame_id = base_frame;
  transformed_marker.color.a = 1.0;
  transformed_marker.action = visualization_msgs::msg::Marker::ADD;
  transformed_marker.lifetime.sec = lifetime_sec;
#endif //CAMERA

#ifdef SIMULATION
  this->declare_parameter("simulation.robots");
  this->declare_parameter("simulation.refresh_frequency");
  robots = this->get_parameter("simulation.robots").as_string_array();
  this->get_parameter_or<int>("simulation.refresh_frequency", refresh_frequency, 5);
#endif // SIMULATION
}

visualization_msgs::msg::Marker Assurancetourix::predict_enemies_pos(visualization_msgs::msg::Marker detectedMarker){
  enemies_markers_on_this_cycle.markers.push_back(detectedMarker);
  for(int i = 0; i< int(last_enemies_markers.markers.size()); i++ ){
    if (last_enemies_markers.markers[i].id == detectedMarker.id){
      visualization_msgs::msg::Marker predictedMarker = detectedMarker;
      predictedMarker.pose.position.x += detectedMarker.pose.position.x - last_enemies_markers.markers[i].pose.position.x;
      predictedMarker.pose.position.y += detectedMarker.pose.position.y - last_enemies_markers.markers[i].pose.position.y;
      predictedMarker.id = detectedMarker.id + 10;
      predictedMarker.header.stamp = this->get_clock()->now();
      predictedMarker.color.r = prediction_color[0];
      predictedMarker.color.g = prediction_color[1];
      predictedMarker.color.b = prediction_color[2];
      predictedMarker.color.a = prediction_color[3];
      return predictedMarker;
    }
  }
  return detectedMarker;
}

#ifdef CAMERA
void Assurancetourix::set_auto_exposure(){
  std_msgs::msg::ColorRGBA color;
  geometry_msgs::msg::Point white_paper_pos;
  white_paper_pos.x = 1.5;
  white_paper_pos.y = 0.55;

  for (_camera_settings.exposure; _camera_settings.exposure < 50; _camera_settings.exposure++){
    get_color_from_position(white_paper_pos, color, 21);
    if (color.r > 135) break;
    else _cap.set(cv::CAP_PROP_EXPOSURE, _camera_settings.exposure);
    std::cout << "color: " << color.b << std::endl;
    std::cout << "exposure: " << _camera_settings.exposure << std::endl;
  }
}

void Assurancetourix::estimate_initial_camera_pose(){
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(this);
  assurancetourix_to_map_transformation.header.frame_id = "map";
  assurancetourix_to_map_transformation.child_frame_id = header_frame_id;

  for (int i = 0; i<10; i++){
    geometry_msgs::msg::Vector3 initial_center_point;
    tf2::Quaternion initial_center_q;
    int sample_nb = 25;
    std::vector<double> roll_vec, pitch_vec, yaw_vec;
    int considered = 0;

    for (int i=0; i<sample_nb; i++){
      get_image();
      _detect_aruco(_anotated);
      estimate_arucos_poses();
      if (center_marker.pose.position.x!=0 && center_marker.pose.position.y!=0 && center_marker.pose.position.z!=0){
        initial_center_point.x += center_marker.pose.position.x;
        initial_center_point.y += center_marker.pose.position.y;
        initial_center_point.z += center_marker.pose.position.z;
        tf2::Quaternion q_tf2;
        tf2::fromMsg(center_marker.pose.orientation, q_tf2);
        tf2::Matrix3x3 m(q_tf2);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        roll_vec.push_back(roll);
        pitch_vec.push_back(pitch);
        yaw_vec.push_back(yaw);
        considered ++;
      }
    }
    tf2::Quaternion q;
    initial_center_q.setRPY(geometrix->mean_angle(roll_vec), geometrix->mean_angle(pitch_vec), geometrix->mean_angle(yaw_vec));
    initial_center_point.x = initial_center_point.x/considered;
    initial_center_point.y = initial_center_point.y/considered;
    initial_center_point.z = initial_center_point.z/considered;

    geometry_msgs::msg::TransformStamped tf_to_center_marker, tf_from_marker_to_camera;

    tf_to_center_marker.transform.translation.x = 1.5;
    tf_to_center_marker.transform.translation.y = 0.75;

    tf_from_marker_to_camera.transform.translation = initial_center_point;
    tf_from_marker_to_camera.transform.rotation = tf2::toMsg(initial_center_q);

    auto camera_from_marker_tf = tf2::transformToKDL(tf_from_marker_to_camera);
    camera_from_marker_tf = camera_from_marker_tf.Inverse();

    geometry_msgs::msg::PoseStamped pose_stamped_camera_from_marker_tf;
    pose_stamped_camera_from_marker_tf.pose = tf2::toMsg(camera_from_marker_tf);

    tf2::Stamped<KDL::Frame> camera_pose_kdl, kdl_tmp_frame;
    tf2::fromMsg(tf2::toMsg(pose_stamped_camera_from_marker_tf), kdl_tmp_frame);
    tf2::doTransform(kdl_tmp_frame, camera_pose_kdl, tf_to_center_marker);

    assurancetourix_to_map_transformation.transform.translation.x = camera_pose_kdl.p[0];
    assurancetourix_to_map_transformation.transform.translation.y = camera_pose_kdl.p[1];
    assurancetourix_to_map_transformation.transform.translation.z = camera_pose_kdl.p[2];
    camera_pose_kdl.M.GetQuaternion(
      assurancetourix_to_map_transformation.transform.rotation.x,
      assurancetourix_to_map_transformation.transform.rotation.y,
      assurancetourix_to_map_transformation.transform.rotation.z,
      assurancetourix_to_map_transformation.transform.rotation.w
    );

    auto camera_pose_kdl_inv = camera_pose_kdl.Inverse();

    assurancetourix_to_map_tf_inv.transform.translation.x = camera_pose_kdl_inv.p[0];
    assurancetourix_to_map_tf_inv.transform.translation.y = camera_pose_kdl_inv.p[1];
    assurancetourix_to_map_tf_inv.transform.translation.z = camera_pose_kdl_inv.p[2];
    camera_pose_kdl_inv.M.GetQuaternion(
      assurancetourix_to_map_tf_inv.transform.rotation.x,
      assurancetourix_to_map_tf_inv.transform.rotation.y,
      assurancetourix_to_map_tf_inv.transform.rotation.z,
      assurancetourix_to_map_tf_inv.transform.rotation.w
    );

    if (assurancetourix_to_map_transformation.transform.rotation.x != 0.0) break;
    else if (i==9) {
      RCLCPP_FATAL(this->get_logger(), "Cannot estimate tf, exiting...");
      exit(-1);
    }
    else RCLCPP_WARN(this->get_logger(), "Cannot estimate tf for the %d time, retrying...", i+1);

  }

  static_tf_broadcaster.sendTransform(assurancetourix_to_map_transformation);

}

void Assurancetourix::init_camera_settings(){
  _cap.open(0, cv::CAP_V4L2);

  this->declare_parameter("camera_settings.width");
  this->declare_parameter("camera_settings.height");
  this->declare_parameter("camera_settings.brightness");
  this->declare_parameter("camera_settings.contrast");
  this->declare_parameter("camera_settings.saturation");
  this->declare_parameter("camera_settings.hue");
  this->declare_parameter("camera_settings.gamma");
  this->declare_parameter("camera_settings.gain");
  this->declare_parameter("camera_settings.backlight_compensation");
  this->declare_parameter("camera_settings.auto_WB");
  this->declare_parameter("camera_settings.exposure");
  this->declare_parameter("camera_settings.sharpness");

  this->get_parameter<int>("camera_settings.width", _camera_settings.width);
  this->get_parameter<int>("camera_settings.height", _camera_settings.height);
  this->get_parameter<int>("camera_settings.brightness", _camera_settings.brightness);
  this->get_parameter<int>("camera_settings.contrast", _camera_settings.contrast);
  this->get_parameter<int>("camera_settings.saturation", _camera_settings.saturation);
  this->get_parameter<int>("camera_settings.hue", _camera_settings.hue);
  this->get_parameter<int>("camera_settings.gamma", _camera_settings.gamma);
  this->get_parameter<int>("camera_settings.gain", _camera_settings.gain);
  this->get_parameter<int>("camera_settings.backlight_compensation", _camera_settings.backlight_compensation);
  this->get_parameter<bool>("camera_settings.auto_WB", _camera_settings.auto_WB);
  this->get_parameter<int>("camera_settings.exposure", _camera_settings.exposure);
  this->get_parameter<int>("camera_settings.sharpness", _camera_settings.sharpness);

  _cap.set(cv::CAP_PROP_FRAME_WIDTH, _camera_settings.width);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _camera_settings.height);
  _cap.set(cv::CAP_PROP_BRIGHTNESS, _camera_settings.brightness);
  _cap.set(cv::CAP_PROP_CONTRAST, _camera_settings.contrast);
  _cap.set(cv::CAP_PROP_SATURATION, _camera_settings.saturation);
  _cap.set(cv::CAP_PROP_HUE, _camera_settings.hue);
  _cap.set(cv::CAP_PROP_GAMMA, _camera_settings.gamma);
  _cap.set(cv::CAP_PROP_GAIN, _camera_settings.gain);
  _cap.set(cv::CAP_PROP_BACKLIGHT, _camera_settings.backlight_compensation);
  _cap.set(cv::CAP_PROP_AUTO_WB, _camera_settings.auto_WB);
  _cap.set(cv::CAP_PROP_EXPOSURE, _camera_settings.exposure);
  _cap.set(cv::CAP_PROP_SHARPNESS , _camera_settings.sharpness);
  _cap.set(cv::CAP_PROP_FPS , 30);
  _cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
  _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
}

void Assurancetourix::get_image() {
  _cap.read(_anotated);
}

// service enabling/disabling aruco detection, disabled by default
// service command line to enable aruco_detection: ros2 service call /enable_aruco_detection std_srvs/srv/SetBool "{data: true}"
void Assurancetourix::handle_aruco_detection_enable(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr request,
                                                    const std_srvs::srv::SetBool::Response::SharedPtr response) {

  if (request->data) {
    timer_ = this->create_wall_timer(0.1s, std::bind(&Assurancetourix::detection_timer_callback_routine, this));
    response->message = "Aruco detection is enabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is enabled");
  } else {
    timer_ = NULL;
    response->message = "Aruco detection is disabled";
    RCLCPP_WARN(this->get_logger(), "Aruco detection is disabled");
  }
  response->success = true;
}

void Assurancetourix::detection_timer_callback_routine() {

#ifdef EXTRALOG // if enabled show the time elapsed to do some actions (capture the image, find the coordonates of the arucos in it and the total)
  // will also show a marker at the assurancetourix origin frame position
  marker.pose.orientation.x = 0.70711;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0.70711;
  marker.pose.orientation.w = 0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.type = aruco_element_type;
  marker.color.r = 255;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.id = 13;

  auto start_camera = std::chrono::high_resolution_clock::now();
  std::ios_base::sync_with_stdio(false);
#endif

  marker.header.stamp = this->get_clock()->now();
  if (marker.header.stamp.nanosec > 70000000) marker.header.stamp.nanosec -= 70000000;
  else {
    marker.header.stamp.sec -= 1;
    marker.header.stamp.nanosec = 1000000000 - 70000000 + marker.header.stamp.nanosec;
  }
  get_image();

#ifdef EXTRALOG
  auto end_camera = std::chrono::high_resolution_clock::now();

  auto start_detection = std::chrono::high_resolution_clock::now();
#endif

  _detect_aruco(_anotated);
  estimate_arucos_poses();

#ifdef EXTRALOG
  auto end_detection = std::chrono::high_resolution_clock::now();
  auto end_total = std::chrono::high_resolution_clock::now();

  double time_taken_camera = std::chrono::duration_cast<std::chrono::nanoseconds>(end_camera - start_camera).count();
  double time_taken_detection = std::chrono::duration_cast<std::chrono::nanoseconds>(end_detection - start_detection).count();
  double time_taken_total = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - start_camera).count();

  time_taken_camera *= 1e-9;
  time_taken_detection *= 1e-9;
  time_taken_total *= 1e-9;

  RCLCPP_INFO(this->get_logger(), "time of camera: %f", time_taken_camera);
  RCLCPP_INFO(this->get_logger(), "time of detection: %f", time_taken_detection);
  RCLCPP_INFO(this->get_logger(), "time of total: %f", time_taken_total);
#endif

  if (!first_image_saved) {
    cv::imwrite("test.jpg", _anotated);
    first_image_saved = true;
  }
  if (show_image) {
    cv::resize(_anotated, _anotated, cv::Size(), 0.4, 0.4, cv::INTER_LINEAR);
    cv::imshow("anotated", _anotated);
    cv::waitKey(3);
  }
}

void Assurancetourix::compute_final_projection(std::vector<std::vector<cv::Point2f>> &coordinates_vector, std::vector<cv::Point2f> &undistort){
  coordinates_vector.push_back({ {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} });
  for(int j=0; j<4; j++){
    coordinates_vector.back()[j] = {
        undistort[j].x * mat_camera_matrix_coeff_fisheye_balanced[0][0] +
        mat_camera_matrix_coeff_fisheye_balanced[0][2],
        undistort[j].y * mat_camera_matrix_coeff_fisheye_balanced[1][1] +
        mat_camera_matrix_coeff_fisheye_balanced[1][2]
      };
  }
}

void Assurancetourix::project_corners_pinhole_to_fisheye(std::vector<std::vector<cv::Point2f>> marker_corners, std::vector<int> detected_ids) {
  _small_marker_corners_projection.clear();
  _huge_marker_corners_projection.clear();
  _small_detected_ids.clear();
  _huge_detected_ids.clear();
  for(int i=0; i<int(detected_ids.size()); i++){
    std::vector<cv::Point2f> undistort;
    cv::fisheye::undistortPoints(marker_corners[i], undistort, _cameraMatrix_fisheye, _distCoeffs_fisheye);
    if (detected_ids[i]==42){
      _center_detected_id.push_back(detected_ids[i]);
      compute_final_projection(_center_corner_projection, undistort);
    }
    else if(detected_ids[i]<50 || detected_ids[i] == geometrix->enemies.arucos[0] ||
            detected_ids[i] == geometrix->enemies.arucos[1] || detected_ids[i] == geometrix->enemies.arucos[2] ||
            detected_ids[i] == geometrix->enemies.arucos[3]){
      _small_detected_ids.push_back(detected_ids[i]);
      compute_final_projection(_small_marker_corners_projection, undistort);
    } else {
      _huge_detected_ids.push_back(detected_ids[i]);
      compute_final_projection(_huge_marker_corners_projection, undistort);
    }
  }
}

void Assurancetourix::_detect_aruco(cv::Mat img) {
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
  std::vector<int> detected_ids;
  cv::aruco::detectMarkers(img, _dictionary, marker_corners, detected_ids, _parameters, rejected_candidates);
  if (detected_ids.size() > 0){
    project_corners_pinhole_to_fisheye(marker_corners, detected_ids);
    if (show_image) {
      // drawDetectedMarkers on the image (activate only for debug)
      cv::aruco::drawDetectedMarkers(_anotated, marker_corners, detected_ids);
    }
  }
}

void Assurancetourix::set_vision_for_rviz(std::vector<double> color, std::vector<double> scale, uint type) {
  marker.type = type;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.scale.x = scale[0];
  marker.scale.y = scale[1];
  marker.scale.z = scale[2];
  transformed_marker.type = type;
  transformed_marker.color.r = color[0];
  transformed_marker.color.g = color[1];
  transformed_marker.color.b = color[2];
  transformed_marker.scale.x = scale[0];
  transformed_marker.scale.y = scale[1];
  transformed_marker.scale.z = scale[2];
}

void Assurancetourix::compute_estimation_markers(std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs,
    visualization_msgs::msg::MarkerArray &marker_array_ennemies, visualization_msgs::msg::MarkerArray &marker_array_allies,
    std::vector<int> detected_ids, visualization_msgs::msg::MarkerArray &markers_camera_relative){
  for (int i = 0; i < int(detected_ids.size()); i++) {

    marker.pose.position.x = tvecs[i].operator[](0);
    marker.pose.position.y = tvecs[i].operator[](1);
    marker.pose.position.z = tvecs[i].operator[](2);

    double angle = norm(rvecs[i]);
    cv::Vec3d axis = rvecs[i] / angle;

    tf2::Quaternion q;
    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

    marker.pose.orientation = tf2::toMsg(q);

    marker.id = detected_ids[i];

    geometry_msgs::msg::PoseStamped tmpPoseIn, tmpPoseOut;
    tmpPoseIn.header = marker.header;
    tmpPoseIn.pose = marker.pose;

    tf2::doTransform<geometry_msgs::msg::PoseStamped>(tmpPoseIn, tmpPoseOut, assurancetourix_to_map_transformation);

    transformed_marker = marker;
    transformed_marker.header = tmpPoseOut.header;
    transformed_marker.header.stamp = marker.header.stamp;
    transformed_marker.pose = tmpPoseOut.pose;

    int marker_type = geometrix->ally_or_enemy(marker.id);

    if ((marker.id <= 5 && 1 <= marker.id)) {
      set_vision_for_rviz(blue_color_aruco, aruco_element_scale, aruco_element_type);
    } else if (marker.id <= 10) {
      set_vision_for_rviz(yellow_color_aruco, aruco_element_scale, aruco_element_type);
    } else if (marker.id <= 50) {
      set_vision_for_rviz(default_color_aruco, game_elements_scale, game_element_type);
    } else {
      set_vision_for_rviz(prediction_color, aruco_element_scale, aruco_element_type);
    }

    if (marker_type == 0) marker_array_allies.markers.push_back(transformed_marker);
    else if (marker_type == 1) marker_array_ennemies.markers.push_back(transformed_marker);

    markers_camera_relative.markers.push_back(marker);
    if (marker.id == 42) center_marker = marker;
  }
}

void Assurancetourix::estimate_arucos_poses() {
  if (_small_detected_ids.size() > 0 || _huge_detected_ids.size() > 0) {
    visualization_msgs::msg::MarkerArray marker_array_ennemies, marker_array_allies, markers_camera_relative;

    if (_center_detected_id.size() > 0){
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(_center_corner_projection, 0.1, _cameraMatrix_pinhole, _distCoeffs_pinhole, rvecs, tvecs);
      compute_estimation_markers(rvecs, tvecs, marker_array_ennemies, marker_array_allies, _center_detected_id, markers_camera_relative);
    }
    if (_small_detected_ids.size() > 0){
      std::vector<int> corners_ids_7, corners_ids_custom_size;
      std::vector<std::vector<cv::Point2f>> corners_7, corners_small_size;
      for (int i = 0; i<(int)_small_detected_ids.size(); i++){
        if (_small_detected_ids[i]<=10) {
          corners_ids_7.push_back(_small_detected_ids[i]);
          corners_7.push_back(_small_marker_corners_projection[i]);
        } else {
          corners_ids_custom_size.push_back(_small_detected_ids[i]);
          corners_small_size.push_back(_small_marker_corners_projection[i]);
        }
      }
      if (corners_ids_7.size() > 0){
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners_7, 0.07, _cameraMatrix_pinhole, _distCoeffs_pinhole, rvecs, tvecs);
        compute_estimation_markers(rvecs, tvecs, marker_array_ennemies, marker_array_allies, corners_ids_7, markers_camera_relative);
      }
      if (corners_ids_custom_size.size() > 0) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners_small_size, 0.067, _cameraMatrix_pinhole, _distCoeffs_pinhole, rvecs, tvecs);
        compute_estimation_markers(rvecs, tvecs, marker_array_ennemies, marker_array_allies, corners_ids_custom_size, markers_camera_relative);
      }
    }
    if (_huge_detected_ids.size() > 0){
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(_huge_marker_corners_projection, huge_aruco_size, _cameraMatrix_pinhole, _distCoeffs_pinhole, rvecs, tvecs);
      compute_estimation_markers(rvecs, tvecs, marker_array_ennemies, marker_array_allies, _huge_detected_ids, markers_camera_relative);
    }

    marker_pub_->publish(markers_camera_relative);
    geometrix->compute_and_send_markers(marker_array_ennemies, marker_array_allies);
  }
}

cv::Point2d Assurancetourix::get_pixels_from_position(geometry_msgs::msg::Point &position){
  geometry_msgs::msg::PointStamped pt_world;
  pt_world.point = position;

  tf2::Stamped<KDL::Vector> kdl_pt_world, kdl_pt_assurancetourix;
  tf2::fromMsg(pt_world, kdl_pt_world);

  tf2::doTransform(kdl_pt_world, kdl_pt_assurancetourix, assurancetourix_to_map_tf_inv);

  geometry_msgs::msg::Point pt_assurancetourix = tf2::toMsg(kdl_pt_assurancetourix).point;

  double arr_pt_assurancetourix[4][1] = {{pt_assurancetourix.x}, {pt_assurancetourix.y}, {pt_assurancetourix.z}, {1.0}};
  cv::Mat mat_pt_assurancetourix(4, 1, CV_64F, arr_pt_assurancetourix);

  double arr_identity[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
  cv::Mat mat_identity(3, 4, CV_64F, arr_identity);

  cv::Mat mat_pixel(3, 1, CV_64F);

  mat_pixel = _cameraMatrix_pinhole * mat_identity * mat_pt_assurancetourix;

  cv::Point2d undistort_pixels(mat_pixel.at<double>(0,0)/mat_pixel.at<double>(2,0), mat_pixel.at<double>(1,0)/mat_pixel.at<double>(2,0));

  undistort_pixels.x = ( undistort_pixels.x - mat_camera_matrix_coeff_fisheye_balanced[0][2] ) / mat_camera_matrix_coeff_fisheye_balanced[0][0];
  undistort_pixels.y = ( undistort_pixels.y - mat_camera_matrix_coeff_fisheye_balanced[1][2] ) / mat_camera_matrix_coeff_fisheye_balanced[1][1];

  std::vector<cv::Point2d> undistort_pixel_vec, distort_pixel_vec;
  undistort_pixel_vec.push_back(undistort_pixels);

  cv::fisheye::distortPoints(undistort_pixel_vec, distort_pixel_vec, _cameraMatrix_fisheye, _distCoeffs_fisheye);

  return distort_pixel_vec[0];
}


void Assurancetourix::get_color_from_position(geometry_msgs::msg::Point &position, std_msgs::msg::ColorRGBA &color, int square_to_check){

  cv::Point2d pixel = get_pixels_from_position(position);

  color.b = 0.0;
  color.g = 0.0;
  color.r = 0.0;
  color.a = 255.0;

  cv::Mat img_color;
  _cap.read(img_color);
  for (int i = 0; i<square_to_check; i++){
    for (int j = 0; j<square_to_check; j++){
      int u = int(pixel.x + i - square_to_check/2);
      int v = int(pixel.y + j - square_to_check/2);
      cv::Vec3b bgr_pixel = img_color.at<cv::Vec3b>(cv::Point(u, v));
      color.b += float(bgr_pixel[0]);
      color.g += float(bgr_pixel[1]);
      color.r += float(bgr_pixel[2]);
    }
  }

  color.b /= pow(square_to_check, 2);
  color.g /= pow(square_to_check, 2);
  color.r /= pow(square_to_check, 2);

  RCLCPP_INFO(this->get_logger(), "pixel at pose x: %f, y: %f, z: %f is red: %f, green: %f, blue: %f", position.x, position.y, position.z, color.r, color.g, color.b);
}

int Assurancetourix::is_goblet_at_position(geometry_msgs::msg::Point &position){
  std_msgs::msg::ColorRGBA color;
  get_color_from_position(position, color);
  if (color.r > color.g * 2.5) return 1; //red
  else if (color.g > color.r * 2.5) return 2; //green
  else return 0; //Nothing
}

void Assurancetourix::reef_goblet_callback(){
  std::vector<int> reef;
  double x_reef = 0.7;
  geometry_msgs::msg::Point gob;
  gob.x = x_reef;
  gob.y = 2.067;
  gob.z = 0.12;
  for (int i = 0; i < 5; i++){
    gob.x = x_reef + i*0.075;
    int color = is_goblet_at_position(gob);
    reef.push_back(color);
    std::cout << "color: " << color << std::endl;
  }
}

void Assurancetourix::compass_orientation_callback(){
  std_msgs::msg::ColorRGBA color;
  geometry_msgs::msg::Point compass_pos;
  compass_pos.x = 1.5;
  compass_pos.y = 1.975;
  compass_pos.z = 0.45;
  get_color_from_position(compass_pos, color);
  std::cout << "color: " << color.r << std::endl;
  std::cout << "color: " << color.g << std::endl;
  std::cout << "color: " << color.b << std::endl;

  std_msgs::msg::String color_str;
  if (color.r + color.g + color.b < 150) {
    color_str.data = "north";
  }
  else {
    color_str.data = "south";
  }
  compass_pub->publish(color_str);
}
#endif // CAMERA

#ifdef SIMULATION
rclcpp::Time Assurancetourix::get_sim_time(std::shared_ptr<webots::Robot> wb_robot) {
  double seconds = 0;
  double nanosec = modf(wb_robot->getTime(), &seconds) * 1e9;
  return rclcpp::Time((uint32_t)seconds, (uint32_t)nanosec);
}

// webots element positioning
void Assurancetourix::simulation_marker_callback() {
  visualization_msgs::msg::MarkerArray marker_array_ennemies, marker_array_allies;
  visualization_msgs::msg::Marker webots_marker;
  webots_marker.type = aruco_element_type;
  webots_marker.color.r = 255;
  webots_marker.color.a = 1;
  webots_marker.scale.x = 0.1;
  webots_marker.scale.y = 0.1;
  webots_marker.scale.z = 0.1;
  webots_marker.header.frame_id = "map";
  int id = 0;

  for (auto robot : robots) {
    double x, y;
    double theta = acos(wb_supervisor->getFromDef(robot)->getOrientation()[0]);

    if (theta < 0) theta = -theta;
    if (wb_supervisor->getFromDef(robot)->getOrientation()[1] > 0) theta = -theta;

    x = wb_supervisor->getFromDef(robot)->getPosition()[0];
    y = 2 - wb_supervisor->getFromDef(robot)->getPosition()[2];

    webots_marker.header.stamp = get_sim_time(wb_supervisor);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    webots_marker.pose.orientation.x = q.x();
    webots_marker.pose.orientation.y = q.y();
    webots_marker.pose.orientation.z = q.z();
    webots_marker.pose.orientation.w = q.w();

    webots_marker.pose.position.x = x;
    webots_marker.pose.position.y = y;

    webots_marker.text = robot;
    webots_marker.id = id;

    marker_array_allies.markers.push_back(webots_marker);
    id++;
  }
  id = 6;
  webots_marker.pose.orientation.x = 0.0;
  webots_marker.pose.orientation.y = 0.0;
  webots_marker.pose.orientation.z = 0.0;
  webots_marker.pose.orientation.w = 1.0;
  webots_marker.header.stamp = this->get_clock()->now();
  if (comeback) {
    comeback_x += 0.1;
    if (comeback_x > 2.5){
      comeback = false;
    }
  } else {
    comeback_x -= 0.1;
    if (comeback_x < 0.5){
      comeback = true;
    }
  }
  webots_marker.pose.position.x = comeback_x;

  webots_marker.pose.position.y = 1;
  webots_marker.text = "test";
  webots_marker.id = id;
  marker_array_ennemies.markers.push_back(webots_marker);
  marker_array_ennemies.markers.push_back(predict_enemies_pos(webots_marker));

  transformed_marker_pub_ennemies_->publish(marker_array_ennemies);
  last_enemies_markers = enemies_markers_on_this_cycle;
  enemies_markers_on_this_cycle.markers.clear();

  transformed_marker_pub_allies_->publish(marker_array_allies);
}
#endif // SIMULATION

Assurancetourix::~Assurancetourix() {
#ifdef CAMERA
  _cap.release();
#endif // CAMERA
  RCLCPP_INFO(this->get_logger(), "Assurancetourix node terminated");
}
