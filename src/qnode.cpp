// src/qnode.cpp
#include "../include/harvest_master/qnode.hpp"

QNode::QNode()
    : current_state_("DISCONNECTED"), detected_crops_count_(0), current_target_index_(0), total_targets_(0), system_ready_(false), vision_ready_(false), motor_ready_(false), planning_ready_(false) {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("harvest_master_gui");

  // Publishers 초기화
  state_pub_ = node_->create_publisher<std_msgs::msg::String>("/system_state", 10);
  camera_trigger_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/camera_trigger", 10);
  joint_velocity_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity", 10);
  path_request_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/path_planning_request", 10);
  cutting_trigger_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cutting_trigger", 10);
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("/system_command", 10);

  // Subscribers 초기화
  crop_detection_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>("/detected_crops", 10, std::bind(&QNode::cropDetectionCallback, this, std::placeholders::_1));

  harvest_order_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/harvest_order", 10, std::bind(&QNode::harvestOrderCallback, this, std::placeholders::_1));

  planned_path_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/planned_path", 10, std::bind(&QNode::plannedPathCallback, this, std::placeholders::_1));

  foundation_pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>("/crop_6d_pose", 10, std::bind(&QNode::foundationPoseCallback, this, std::placeholders::_1));

  robot_status_sub_ = node_->create_subscription<std_msgs::msg::String>("/robot_status", 10, std::bind(&QNode::robotStatusCallback, this, std::placeholders::_1));

  vision_status_sub_ = node_->create_subscription<std_msgs::msg::String>("/vision_status", 10, std::bind(&QNode::visionStatusCallback, this, std::placeholders::_1));

  motor_status_sub_ = node_->create_subscription<std_msgs::msg::String>("/motor_status", 10, std::bind(&QNode::motorStatusCallback, this, std::placeholders::_1));

  planning_status_sub_ = node_->create_subscription<std_msgs::msg::String>("/planning_status", 10, std::bind(&QNode::planningStatusCallback, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(30);  // 30Hz
  current_state_ = "READY";
  Q_EMIT stateChanged(current_state_);
  Q_EMIT logMessage("ROS 2 시스템 초기화 완료");

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::startHarvestSequence() {
  auto msg = std_msgs::msg::String();
  msg.data = "START_HARVEST";
  command_pub_->publish(msg);

  current_state_ = "INITIALIZING";
  Q_EMIT stateChanged(current_state_);
  Q_EMIT logMessage("수확 시퀀스 시작");
}

void QNode::stopHarvestSequence() {
  auto msg = std_msgs::msg::String();
  msg.data = "STOP_HARVEST";
  command_pub_->publish(msg);

  current_state_ = "STOPPED";
  Q_EMIT stateChanged(current_state_);
  Q_EMIT logMessage("수확 시퀀스 중지");
}

void QNode::emergencyStop() {
  auto msg = std_msgs::msg::String();
  msg.data = "EMERGENCY_STOP";
  command_pub_->publish(msg);

  current_state_ = "EMERGENCY";
  Q_EMIT stateChanged(current_state_);
  Q_EMIT logMessage("긴급 정지 실행!");
  Q_EMIT errorOccurred("긴급 정지가 실행되었습니다.");
}

void QNode::resetSystem() {
  auto msg = std_msgs::msg::String();
  msg.data = "RESET_SYSTEM";
  command_pub_->publish(msg);

  // 내부 상태 초기화
  detected_crops_count_ = 0;
  current_target_index_ = 0;
  total_targets_ = 0;
  detected_crops_.clear();
  harvest_order_.clear();
  last_error_.clear();

  current_state_ = "READY";
  Q_EMIT stateChanged(current_state_);
  Q_EMIT logMessage("시스템 초기화 완료");
  Q_EMIT progressUpdated(0, 0);
}

void QNode::triggerVisionDetection() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  camera_trigger_pub_->publish(msg);
  Q_EMIT logMessage("비전 감지 트리거 전송");
}

void QNode::moveToPosition(double x, double y, double z, double rx, double ry, double rz) {
  auto msg = geometry_msgs::msg::Pose();
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;

  // Euler angles to quaternion (simplified)
  msg.orientation.x = rx;
  msg.orientation.y = ry;
  msg.orientation.z = rz;
  msg.orientation.w = 1.0;

  path_request_pub_->publish(msg);
  Q_EMIT logMessage(QString("위치 이동 요청: (%.3f, %.3f, %.3f)").arg(x).arg(y).arg(z));
}

void QNode::setJointVelocities(const std::vector<double>& velocities) {
  if (velocities.size() != 6) {
    Q_EMIT errorOccurred("관절 속도는 6개 값이 필요합니다.");
    return;
  }

  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = velocities;
  joint_velocity_pub_->publish(msg);
  Q_EMIT logMessage("관절 속도 명령 전송");
}

void QNode::activateCuttingTool() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  cutting_trigger_pub_->publish(msg);
  Q_EMIT logMessage("절단 도구 활성화");
}

void QNode::requestPathPlanning(const geometry_msgs::msg::Pose& target) {
  path_request_pub_->publish(target);
  Q_EMIT logMessage("경로 계획 요청");
}

// 콜백 함수들
void QNode::cropDetectionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  detected_crops_ = msg->poses;
  detected_crops_count_ = msg->poses.size();

  // 익은 참외와 안 익은 참외 분류 (간단히 z 좌표로 구분)
  int ripe_count = 0;
  int unripe_count = 0;

  for (const auto& pose : msg->poses) {
    if (pose.position.z > 0.5) {  // 임의 기준
      ripe_count++;
    } else {
      unripe_count++;
    }
  }

  Q_EMIT cropsDetected(detected_crops_count_);
  Q_EMIT detectionComplete(ripe_count, unripe_count);
  Q_EMIT logMessage(QString("참외 감지 완료: 총 %1개 (익은 것: %2개, 덜 익은 것: %3개)").arg(detected_crops_count_).arg(ripe_count).arg(unripe_count));
}

void QNode::harvestOrderCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  harvest_order_.clear();
  QStringList orderList;

  for (size_t i = 0; i < msg->data.size(); ++i) {
    int cropIndex = static_cast<int>(msg->data[i]);
    harvest_order_.push_back(cropIndex);
    orderList << QString("참외 %1").arg(cropIndex + 1);
  }

  total_targets_ = harvest_order_.size();
  Q_EMIT harvestOrderUpdated(orderList);
  Q_EMIT logMessage(QString("수확 순서 계산 완료: %1개 목표").arg(total_targets_));
}

void QNode::plannedPathCallback(const std_msgs::msg::Float64MultiArray::SharedPtr /*msg*/) {
  Q_EMIT pathPlanningComplete();
  Q_EMIT logMessage("경로 계획 완료");
}

void QNode::foundationPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  current_target_pose_ = *msg;
  Q_EMIT foundationPoseComplete();
  Q_EMIT logMessage(QString("6D Pose 추정 완료: (%.3f, %.3f, %.3f)").arg(msg->position.x).arg(msg->position.y).arg(msg->position.z));
}

void QNode::robotStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  Q_EMIT robotStatusUpdated(QString::fromStdString(msg->data));

  if (msg->data == "POSITION_REACHED") {
    Q_EMIT positionReached();
  } else if (msg->data == "CUTTING_COMPLETE") {
    current_target_index_++;
    Q_EMIT cuttingComplete();
    Q_EMIT progressUpdated(current_target_index_, total_targets_);
  }
}

void QNode::visionStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  vision_ready_ = (msg->data == "READY");
  Q_EMIT visionSystemReady();
  Q_EMIT logMessage(QString("비전 시스템: %1").arg(QString::fromStdString(msg->data)));
}

void QNode::motorStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  motor_ready_ = (msg->data == "READY");
  Q_EMIT motorSystemReady();
  Q_EMIT logMessage(QString("모터 시스템: %1").arg(QString::fromStdString(msg->data)));
}

void QNode::planningStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  planning_ready_ = (msg->data == "READY");
  Q_EMIT pathPlanningReady();
  Q_EMIT logMessage(QString("경로 계획: %1").arg(QString::fromStdString(msg->data)));
}