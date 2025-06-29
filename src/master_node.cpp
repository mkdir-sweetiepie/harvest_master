// master_node.cpp
#include "../include/harvest_master/master_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

MasterNode::MasterNode() : Node("master_node") {
  initializePublishers();
  initializeSubscribers();
  initializeServiceClients();
  initializeVariables();

  // 상태머신 주기적 실행을 위한 타이머 (100ms 주기)
  timer_ = this->create_wall_timer(100ms, std::bind(&MasterNode::stateMachineCallback, this));

  RCLCPP_INFO(this->get_logger(), "MasterNode 초기화 완료");
}

MasterNode::~MasterNode() {
  // 리소스 정리
  if (timer_) {
    timer_->cancel();
  }

  // 그리퍼 서비스 클라이언트 정리
  gripper_open_client_.reset();
  gripper_close_client_.reset();

  RCLCPP_INFO(this->get_logger(), "MasterNode 소멸자 실행 완료");
}

// ===== 초기화 함수들 =====

void MasterNode::initializePublishers() {
  // 경로 계획 노드로 다양한 명령 전송
  start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/start_command", 10);
  goal_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/goal_command", 10);
  path_pub_ = this->create_publisher<vision_msgs::msg::HarvestOrdering>("/harvest_order", 10);
  // 각 모듈 활성화 신호 전송
  foundation_pub_ = this->create_publisher<std_msgs::msg::Bool>("/foundation_activate", 10);
  yolo_pub_ = this->create_publisher<std_msgs::msg::Bool>("/yolo_activate", 10);
  shutdown_pub_ = this->create_publisher<std_msgs::msg::String>("/shutdown_signal", 10);
}

void MasterNode::initializeSubscribers() {
  // 각 노드들로부터 상태 정보 수신
  movement_sub_ = this->create_subscription<std_msgs::msg::Bool>("/movement_complete", 10, std::bind(&MasterNode::movementCallback, this, std::placeholders::_1));
  tsp_sub_ = this->create_subscription<vision_msgs::msg::HarvestOrdering>("/harvest_ordering/result2", 10, std::bind(&MasterNode::tspCallback, this, std::placeholders::_1));
  foundation_sub_ = this->create_subscription<vision_msgs::msg::CropPose>("/CropPose/obj/result", 10, std::bind(&MasterNode::foundationCallback, this, std::placeholders::_1));
}

void MasterNode::initializeServiceClients() {
  // 그리퍼 제어를 위한 서비스 클라이언트
  gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>("gripper_open_command");
  gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>("gripper_close_command");
}

void MasterNode::initializeVariables() {
  // 상태머신 초기 상태 설정
  current_state_.store(MasterState::INIT);
  current_fruit_index_.store(0);
  total_fruit_count_ = 8;

  // 상태별 플래그 초기화
  resetStateFlags();

  // 로봇 위치 좌표 설정
  start_ = true;
  initial_goal_.x = -0.126;
  initial_goal_.y = 0.0;
  initial_goal_.z = 0.782;

  // 상태 플래그 초기화 (모든 노드 준비 상태로 설정)
  nodes_ready_ = true;
  yolo_ready_ = true;
  tsp_ready_ = true;
  foundation_ready_ = true;
  path_ready_ = true;
  movement_complete_.store(false);
  tsp_complete_.store(false);
  foundation_complete_.store(false);
  gripper_open_complete_.store(false);
  gripper_close_complete_.store(false);

  state_start_time_ = this->now();
}

void MasterNode::resetStateFlags() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  initial_command_sent_ = false;
  recognition_started_ = false;
  move_command_sent_ = false;
  foundation_started_ = false;
  gripper_opened_ = false;
  cutting_move_sent_ = false;
  gripper_closed_ = false;
  back_move_sent_ = false;
  next_move_sent_ = false;
  return_sent_ = false;
  error_count_ = 0;
}

// ===== 메인 상태 머신 =====
void MasterNode::stateMachineCallback() {
  MasterState current = current_state_.load();

  // 현재 상태에 따른 동작 수행
  switch (current) {
    case MasterState::INIT:
      handleInitState();
      break;
    case MasterState::INITIAL_MOVE:
      handleInitialMoveState();
      break;
    case MasterState::FRUIT_RECOGNITION:
      handleFruitRecognitionState();
      break;
    case MasterState::TSP_PROCESSING:
      handleTspProcessingState();
      break;
    case MasterState::HARVEST_LOOP:
      handleHarvestLoopState();
      break;
    case MasterState::FOUNDATION_PROCESSING:
      handleFoundationProcessingState();
      break;
    case MasterState::GRIPPER_OPEN:
      handleGripperOpenState();
      break;
    case MasterState::MOVE_TO_CUTTING:
      handleMoveToCuttingState();
      break;
    case MasterState::GRIPPER_CLOSE:
      handleGripperCloseState();
      break;
    case MasterState::MOVE_BACK:
      handleMoveBackState();
      break;
    case MasterState::MOVE_TO_NEXT:
      handleMoveToNextState();
      break;
    case MasterState::RETURN_HOME:
      handleReturnHomeState();
      break;
    case MasterState::ERROR_STATE:
      handleErrorState();
      break;
    case MasterState::SHUTDOWN:
      handleShutdownState();
      break;
  }
}

// ===== 상태별 핸들러 함수들 =====

void MasterNode::handleInitState() {
  RCLCPP_INFO(this->get_logger(), "INIT : 모든 노드 준비 상태 확인 중…");

  if (nodes_ready_) {
    changeState(MasterState::INITIAL_MOVE);
  }
}

void MasterNode::handleInitialMoveState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!initial_command_sent_ && start_) {
    RCLCPP_INFO(this->get_logger(), "INITIAL_MOVE : 초기 포즈로 이동 명령 전송");
    sendInitialCommand();
    initial_command_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "INITIAL_MOVE : 초기 이동 완료");
    current_position_ = initial_goal_;
    movement_complete_.store(false);
    start_ = false;
    changeState(MasterState::FRUIT_RECOGNITION);
  }
}

void MasterNode::handleFruitRecognitionState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!recognition_started_) {
    RCLCPP_INFO(this->get_logger(), "FRUIT_RECOGNITION : 과일 인식 시작");
    activateYolo();
    recognition_started_ = true;
  }

  if (tsp_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "FRUIT_RECOGNITION : 과일 인식 및 TSP 완료");
    tsp_complete_.store(false);
    current_fruit_index_.store(0);
    changeState(MasterState::HARVEST_LOOP);
  }
}

void MasterNode::handleTspProcessingState() {
  // 이 상태는 더 이상 사용하지 않음 (YOLO에서 직접 TSP로 연결)
  changeState(MasterState::HARVEST_LOOP);
}

void MasterNode::handleHarvestLoopState() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  int current_index = current_fruit_index_.load();

  // 모든 과일 수확 완료 검사
  if (static_cast<size_t>(current_index) >= priority_list_.size()) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP : 모든 과일 수확 완료");
    changeState(MasterState::RETURN_HOME);
    return;
  }

  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (!move_command_sent_) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP : %d번째 과일로 이동(인덱스 : %d)", current_index + 1, priority_list_[current_index]);
    sendTspCommand();
    move_command_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP : 과일 위치 도착");
    current_position_ = fruit_positions_[priority_list_[current_index]];
    movement_complete_.store(false);
    changeState(MasterState::FOUNDATION_PROCESSING);
  }
}

void MasterNode::handleFoundationProcessingState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!foundation_started_) {
    RCLCPP_INFO(this->get_logger(), "FOUNDATION : 6D 포즈 추정 시작");
    activateFoundation();
    foundation_started_ = true;
  }

  if (foundation_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "FOUNDATION : 6D 포즈 추정 완료");
    foundation_complete_.store(false);
    changeState(MasterState::GRIPPER_OPEN);
  }
}

void MasterNode::handleGripperOpenState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!gripper_opened_) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_OPEN : 그리퍼 벌리기");
    sendGripperCommand(true);  // true = open
    gripper_opened_ = true;
  }

  if (gripper_open_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_OPEN : 그리퍼 열기 완료");
    gripper_open_complete_.store(false);
    changeState(MasterState::MOVE_TO_CUTTING);
  }
}

void MasterNode::handleMoveToCuttingState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!cutting_move_sent_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_CUTTING : 절단점으로 이동");
    sendFoundationCommand();
    cutting_move_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_CUTTING : 절단점 도착");
    current_position_ = cutting_point_;
    movement_complete_.store(false);
    changeState(MasterState::GRIPPER_CLOSE);
  }
}

void MasterNode::handleGripperCloseState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!gripper_closed_) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_CLOSE : 그리퍼 닫기(수확)");
    sendGripperCommand(false);  // false = close
    gripper_closed_ = true;
  }

  if (gripper_close_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_CLOSE : 그리퍼 닫기 완료");
    gripper_close_complete_.store(false);
    changeState(MasterState::MOVE_BACK);
  }
}

void MasterNode::handleMoveBackState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!back_move_sent_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_BACK : 과일 위치로 복귀");
    sendTspCommand();  // 현재 과일 위치로 복귀
    back_move_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "MOVE_BACK : 과일 위치 복귀 완료");
    int current_index = current_fruit_index_.load();
    current_position_ = fruit_positions_[priority_list_[current_index]];
    movement_complete_.store(false);
    current_fruit_index_.store(current_index + 1);  // 다음 과일 인덱스로 증가
    changeState(MasterState::MOVE_TO_NEXT);
  }
}

void MasterNode::handleMoveToNextState() {
  int current_index = current_fruit_index_.load();

  // 모든 과일 처리 완료 검사
  if (static_cast<size_t>(current_index) >= priority_list_.size()) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT : 모든 과일 처리 완료");
    changeState(MasterState::RETURN_HOME);
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!next_move_sent_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT : 다음 과일로 이동");
    sendTspCommand();
    next_move_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT : 다음 과일 위치 도착");
    current_position_ = fruit_positions_[priority_list_[current_index]];
    movement_complete_.store(false);
    changeState(MasterState::FOUNDATION_PROCESSING);
  }
}

void MasterNode::handleReturnHomeState() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!return_sent_) {
    RCLCPP_INFO(this->get_logger(), "RETURN_HOME : 시작점으로 복귀");
    sendReturnHomeCommand();
    return_sent_ = true;
  }

  if (movement_complete_.load()) {
    RCLCPP_INFO(this->get_logger(), "RETURN_HOME : 시작점 복귀 완료");
    movement_complete_.store(false);
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleErrorState() {
  error_count_++;

  RCLCPP_ERROR(this->get_logger(), "ERROR_STATE : 에러 상태 처리 중... (횟수: %d)", error_count_);

  if (error_count_ >= 3) {
    RCLCPP_ERROR(this->get_logger(), "최대 에러 횟수 초과, 시스템 종료");
    changeState(MasterState::SHUTDOWN);
  } else {
    // 에러 복구 시도 - 홈으로 복귀
    RCLCPP_INFO(this->get_logger(), "에러 복구 시도 - 홈으로 복귀");
    changeState(MasterState::RETURN_HOME);
  }
}

void MasterNode::handleShutdownState() {
  RCLCPP_INFO(this->get_logger(), "SHUTDOWN : 모든 노드 종료 신호 전송");
  sendShutdownSignal();

  // 리소스 정리
  if (timer_) {
    timer_->cancel();
  }

  // 잠시 대기 후 종료
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  rclcpp::shutdown();
}

// ===== 유틸리티 함수들 =====

void MasterNode::changeState(MasterState new_state) {
  MasterState old_state = current_state_.load();
  current_state_.store(new_state);
  state_start_time_ = this->now();
  resetStateFlags();

  RCLCPP_INFO(this->get_logger(), "상태 전환: %d -> %d", static_cast<int>(old_state), static_cast<int>(new_state));
}

// ===== 로봇 명령 전송 함수들 (용도별 구분) =====

void MasterNode::sendInitialCommand() {
  std::vector<double> goal_array = {initial_goal_.x, initial_goal_.y, initial_goal_.z};

  vision_msgs::msg::HarvestOrdering default_harvest;
  default_harvest.header.stamp = this->now();
  default_harvest.header.frame_id = "base_link";
  default_harvest.total_objects = 1;

  vision_msgs::msg::DetectedCrop default_crop;
  default_crop.id = 1;
  default_crop.x = 2.0;
  default_crop.y = 0.0;
  default_crop.z = 0.0;
  default_harvest.objects.push_back(default_crop);
  default_harvest.crop_ids.push_back(1);

  sendStartCommand(true);
  sendGoalCommand(goal_array);
  sendPathCommand(default_harvest);

  RCLCPP_INFO(this->get_logger(), "Initial command sent - Goal: (%.3f, %.3f, %.3f)", initial_goal_.x, initial_goal_.y, initial_goal_.z);
}

void MasterNode::sendTspCommand() {
  int current_index = current_fruit_index_.load();

  std::lock_guard<std::mutex> lock(data_mutex_);

  geometry_msgs::msg::Point current_fruit_goal = fruit_positions_[priority_list_[current_index]];
  std::vector<double> goal_array = {current_fruit_goal.x, current_fruit_goal.y, current_fruit_goal.z};

  // TSP에서 받은 HarvestOrdering 메시지 전송
  vision_msgs::msg::HarvestOrdering tsp_harvest;
  tsp_harvest.header.stamp = this->now();
  tsp_harvest.header.frame_id = "base_link";
  tsp_harvest.total_objects = fruit_positions_.size();

  for (size_t i = 0; i < fruit_positions_.size(); ++i) {
    vision_msgs::msg::DetectedCrop crop;
    crop.id = static_cast<uint32_t>(i + 1);
    crop.x = fruit_positions_[i].x;
    crop.y = fruit_positions_[i].y;
    crop.z = fruit_positions_[i].z;
    tsp_harvest.objects.push_back(crop);
  }

  for (int priority : priority_list_) {
    tsp_harvest.crop_ids.push_back(static_cast<uint32_t>(priority + 1));
  }

  sendStartCommand(true);
  sendGoalCommand(goal_array);
  sendPathCommand(tsp_harvest);

  RCLCPP_INFO(this->get_logger(), "TSP command sent - Goal: (%.3f, %.3f, %.3f)", current_fruit_goal.x, current_fruit_goal.y, current_fruit_goal.z);
}

void MasterNode::sendFoundationCommand() {
  std::vector<double> goal_array = {cutting_point_.x, cutting_point_.y, cutting_point_.z};

  std::lock_guard<std::mutex> lock(data_mutex_);

  // 현재 수확 중인 과일 정보로 HarvestOrdering 생성
  vision_msgs::msg::HarvestOrdering foundation_harvest;
  foundation_harvest.header.stamp = this->now();
  foundation_harvest.header.frame_id = "base_link";
  foundation_harvest.total_objects = fruit_positions_.size();

  for (size_t i = 0; i < fruit_positions_.size(); ++i) {
    vision_msgs::msg::DetectedCrop crop;
    crop.id = static_cast<uint32_t>(i + 1);
    crop.x = fruit_positions_[i].x;
    crop.y = fruit_positions_[i].y;
    crop.z = fruit_positions_[i].z;
    foundation_harvest.objects.push_back(crop);
  }

  for (int priority : priority_list_) {
    foundation_harvest.crop_ids.push_back(static_cast<uint32_t>(priority + 1));
  }

  sendStartCommand(true);
  sendGoalCommand(goal_array);
  sendPathCommand(foundation_harvest);

  RCLCPP_INFO(this->get_logger(), "Foundation command sent - Goal: (%.3f, %.3f, %.3f)", cutting_point_.x, cutting_point_.y, cutting_point_.z);
}

void MasterNode::sendReturnHomeCommand() {
  std::vector<double> goal_array = {initial_goal_.x, initial_goal_.y, initial_goal_.z};

  vision_msgs::msg::HarvestOrdering default_harvest;
  default_harvest.header.stamp = this->now();
  default_harvest.header.frame_id = "base_link";
  default_harvest.total_objects = 1;

  vision_msgs::msg::DetectedCrop default_crop;
  default_crop.id = 1;
  default_crop.x = 2.0;
  default_crop.y = 0.0;
  default_crop.z = 0.0;
  default_harvest.objects.push_back(default_crop);
  default_harvest.crop_ids.push_back(1);

  sendStartCommand(true);
  sendGoalCommand(goal_array);
  sendPathCommand(default_harvest);

  RCLCPP_INFO(this->get_logger(), "Return home command sent - Goal: (%.3f, %.3f, %.3f)", initial_goal_.x, initial_goal_.y, initial_goal_.z);
}

// ===== 개별 전송 함수들 (private) =====

void MasterNode::sendStartCommand(bool start_signal) {
  auto msg = std_msgs::msg::Bool();
  msg.data = start_signal;
  start_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Start command sent: %s", start_signal ? "true" : "false");
}

void MasterNode::sendGoalCommand(const std::vector<double>& goal_array) {
  if (goal_array.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "목표 좌표 배열 크기가 잘못됨: %zu", goal_array.size());
    return;
  }

  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = goal_array;
  goal_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Goal command sent: [%.3f, %.3f, %.3f]", goal_array[0], goal_array[1], goal_array[2]);
}

void MasterNode::sendPathCommand(const vision_msgs::msg::HarvestOrdering& harvest_order) {
  path_pub_->publish(harvest_order);
  RCLCPP_DEBUG(this->get_logger(), "Path command sent: %u crops, %zu priorities", harvest_order.total_objects, harvest_order.crop_ids.size());
}

void MasterNode::sendGripperCommand(bool open) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto client = open ? gripper_open_client_ : gripper_close_client_;
  std::string action = open ? "열기" : "닫기";

  // 서비스 가용성 확인
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "그리퍼 %s 서비스를 사용할 수 없습니다", action.c_str());
    return;
  }

  // 비동기 서비스 호출 및 응답 처리 설정
  auto future = client->async_send_request(request);

  if (open) {
    future.wait_for(std::chrono::seconds(1));
    if (future.valid()) {
      auto response = future.get();
      if (response->success) {
        gripper_open_complete_.store(true);
        RCLCPP_INFO(this->get_logger(), "그리퍼 열기 성공");
      } else {
        RCLCPP_ERROR(this->get_logger(), "그리퍼 열기 실패: %s", response->message.c_str());
      }
    }
  } else {
    future.wait_for(std::chrono::seconds(1));
    if (future.valid()) {
      auto response = future.get();
      if (response->success) {
        gripper_close_complete_.store(true);
        RCLCPP_INFO(this->get_logger(), "그리퍼 닫기 성공");
      } else {
        RCLCPP_ERROR(this->get_logger(), "그리퍼 닫기 실패: %s", response->message.c_str());
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "그리퍼 %s 명령 전송됨", action.c_str());
}

// ===== 모듈 활성화 함수들 =====

void MasterNode::activateYolo() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  yolo_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "YOLO 모듈 활성화");
}

void MasterNode::activateFoundation() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  foundation_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Foundation 모듈 활성화");
}

void MasterNode::sendShutdownSignal() {
  auto msg = std_msgs::msg::String();
  msg.data = "shutdown";
  shutdown_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "종료 신호 전송");
}

// ===== 콜백 함수들 =====

void MasterNode::movementCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  movement_complete_.store(msg->data);
  if (msg->data) {
    RCLCPP_INFO(this->get_logger(), "이동 완료 신호 수신");
  }
}

void MasterNode::tspCallback(const vision_msgs::msg::HarvestOrdering::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "TSP 결과 수신 : %u개 과일", msg->total_objects);

  if (msg->total_objects == 0) {
    RCLCPP_WARN(this->get_logger(), "감지된 과일이 없습니다");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  // 과일 위치 저장
  fruit_positions_.clear();
  for (const auto& crop : msg->objects) {
    geometry_msgs::msg::Point point;
    point.x = crop.x;
    point.y = crop.y;
    point.z = crop.z;

    fruit_positions_.push_back(point);
    RCLCPP_INFO(this->get_logger(), "과일 %u: (%.3f, %.3f, %.3f)", crop.id, crop.x, crop.y, crop.z);
  }

  // 우선순위 리스트 저장 (ID를 인덱스로 변환)
  priority_list_.clear();
  for (uint32_t crop_id : msg->crop_ids) {
    int index = static_cast<int>(crop_id - 1);  // ID는 1부터 시작하므로 인덱스로 변환
    if (index >= 0 && static_cast<size_t>(index) < fruit_positions_.size()) {
      priority_list_.push_back(index);
    } else {
      RCLCPP_WARN(this->get_logger(), "유효하지 않은 작물 ID: %u", crop_id);
    }
  }

  if (priority_list_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "유효한 수확 순서가 없습니다");
    return;
  }

  tsp_complete_.store(true);
  RCLCPP_INFO(this->get_logger(), "TSP 우선순위 수신 : %zu개 순서", priority_list_.size());
}

void MasterNode::foundationCallback(const vision_msgs::msg::CropPose::SharedPtr msg) {
  geometry_msgs::msg::Point new_cutting_point;
  new_cutting_point.x = msg->x;
  new_cutting_point.y = msg->y;
  new_cutting_point.z = msg->z;

  cutting_point_ = new_cutting_point;
  foundation_complete_.store(true);
  RCLCPP_INFO(this->get_logger(), "절단점 수신 : (%.3f, %.3f, %.3f)", cutting_point_.x, cutting_point_.y, cutting_point_.z);
}

// ===== Main 함수 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MasterNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "예외 발생: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}