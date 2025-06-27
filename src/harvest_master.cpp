// master_node.cpp
#include "harvest_master/master_node.hpp"

MasterNode::MasterNode() : Node("master_node") {
  initializePublishers();
  initializeSubscribers();
  initializeServiceClients();
  initializeVariables();

  timer_ = this->create_wall_timer(100ms, std::bind(&MasterNode::stateMachineCallback, this));
}

void MasterNode::initializePublishers() {
  path_pub_ = this->create_publisher<vision_msgs::msg::HarvestOrdering>("/harvest_order", 10);
  foundation_pub_ = this->create_publisher<std_msgs::msg::Bool>("/foundation_activate", 10);
  yolo_pub_ = this->create_publisher<std_msgs::msg::Bool>("/yolo_activate", 10);
  tsp_pub_ = this->create_publisher<std_msgs::msg::Bool>("/tsp_activate", 10);
  map_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/fruit_positions_to_map", 10);
  shutdown_pub_ = this->create_publisher<std_msgs::msg::String>("/shutdown_signal", 10);
}

void MasterNode::initializeSubscribers() {
  node_status_sub_ = this->create_subscription<std_msgs::msg::String>("/node_ready", 10, std::bind(&MasterNode::nodeStatusCallback, this, std::placeholders::_1));
  movement_sub_ = this->create_subscription<std_msgs::msg::Bool>("/movement_complete", 10, std::bind(&MasterNode::movementCallback, this, std::placeholders::_1));
  fruit_detection_sub_ = this->create_subscription<vision_msgs::msg::DetectedCropArray>("/detected_crops", 10, std::bind(&MasterNode::fruitDetectionCallback, this, std::placeholders::_1));
  tsp_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/tsp_result", 10, std::bind(&MasterNode::tspCallback, this, std::placeholders::_1));
  foundation_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/cutting_point", 10, std::bind(&MasterNode::foundationCallback, this, std::placeholders::_1));
  arrival_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arrival_complete", 10, std::bind(&MasterNode::arrivalCallback, this, std::placeholders::_1));
}

void MasterNode::initializeServiceClients() {
  gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>("gripper_open_command");
  gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>("gripper_close_command");
}

void MasterNode::initializeVariables() {
  current_state_ = MasterState::INIT;
  current_fruit_index_ = 0;
  total_fruit_count_ = 8;  // 전시회 때는 10개

  // 시작점과 초기 목표점 설정
  start_position_.x = 0.14;
  start_position_.y = 0.0;
  start_position_.z = 1.005;

  initial_goal_.x = -0.126;
  initial_goal_.y = 0.0;
  initial_goal_.z = 0.782;

  current_position_ = start_position_;

  // 플래그 초기화
  nodes_ready_ = false;
  movement_complete_ = false;
  recognition_complete_ = false;
  tsp_complete_ = false;
  foundation_complete_ = false;
  arrival_complete_ = false;

  state_start_time_ = this->now();
}

// ===== 메인 상태 머신 =====
void MasterNode::stateMachineCallback() {
  auto current_time = this->now();
  auto elapsed = (current_time - state_start_time_).seconds();

  switch (current_state_) {
    case MasterState::INIT:
      handleInitState();
      break;

    case MasterState::INITIAL_MOVE:
      handleInitialMoveState(elapsed);
      break;

    case MasterState::FRUIT_RECOGNITION:
      handleFruitRecognitionState(elapsed);
      break;

    case MasterState::TSP_PROCESSING:
      handleTspProcessingState(elapsed);
      break;

    case MasterState::HARVEST_LOOP:
      handleHarvestLoopState(elapsed);
      break;

    case MasterState::FOUNDATION_PROCESSING:
      handleFoundationProcessingState(elapsed);
      break;

    case MasterState::GRIPPER_OPEN:
      handleGripperOpenState(elapsed);
      break;

    case MasterState::MOVE_TO_CUTTING:
      handleMoveToCuttingState(elapsed);
      break;

    case MasterState::GRIPPER_CLOSE:
      handleGripperCloseState(elapsed);
      break;

    case MasterState::MOVE_BACK:
      handleMoveBackState(elapsed);
      break;

    case MasterState::MOVE_TO_NEXT:
      handleMoveToNextState(elapsed);
      break;

    case MasterState::RETURN_HOME:
      handleReturnHomeState(elapsed);
      break;

    case MasterState::SHUTDOWN:
      handleShutdownState();
      break;
  }
}

// ===== 상태별 핸들러 함수들 =====

void MasterNode::handleInitState() {
  RCLCPP_INFO(this->get_logger(), "INIT: 모든 노드 준비 상태 확인 중...");
  if (nodes_ready_) {
    changeState(MasterState::INITIAL_MOVE);
  }
}

void MasterNode::handleInitialMoveState(double elapsed) {
  static bool initial_command_sent = false;

  if (!initial_command_sent) {
    RCLCPP_INFO(this->get_logger(), "INITIAL_MOVE: 초기 위치로 이동 명령 전송");
    sendPathCommand(std::vector<geometry_msgs::msg::Point>());  // 빈 과일 리스트로 초기 이동
    initial_command_sent = true;
  }

  if (movement_complete_) {
    RCLCPP_INFO(this->get_logger(), "INITIAL_MOVE: 초기 이동 완료");
    current_position_ = initial_goal_;
    initial_command_sent = false;
    movement_complete_ = false;
    changeState(MasterState::FRUIT_RECOGNITION);
  }

  if (elapsed > 30.0) {  // 30초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "INITIAL_MOVE: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleFruitRecognitionState(double elapsed) {
  static bool recognition_started = false;

  if (!recognition_started) {
    RCLCPP_INFO(this->get_logger(), "FRUIT_RECOGNITION: 과일 인식 시작");
    activateYolo();
    recognition_started = true;
  }

  if (recognition_complete_) {
    RCLCPP_INFO(this->get_logger(), "FRUIT_RECOGNITION: 과일 인식 완료");
    recognition_started = false;
    recognition_complete_ = false;
    changeState(MasterState::TSP_PROCESSING);
  }

  if (elapsed > 60.0) {  // 60초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "FRUIT_RECOGNITION: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleTspProcessingState(double elapsed) {
  static bool tsp_started = false;

  if (!tsp_started) {
    RCLCPP_INFO(this->get_logger(), "TSP_PROCESSING: TSP 최적화 시작");
    activateTsp();
    tsp_started = true;
  }

  if (tsp_complete_) {
    RCLCPP_INFO(this->get_logger(), "TSP_PROCESSING: TSP 최적화 완료");
    sendFruitPositionsToMap();
    tsp_started = false;
    tsp_complete_ = false;
    current_fruit_index_ = 0;
    changeState(MasterState::HARVEST_LOOP);
  }

  if (elapsed > 30.0) {  // 30초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "TSP_PROCESSING: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleHarvestLoopState(double elapsed) {
  if (current_fruit_index_ >= priority_list_.size()) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP: 모든 과일 수확 완료");
    changeState(MasterState::RETURN_HOME);
    return;
  }

  static bool move_command_sent = false;

  if (!move_command_sent) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP: %d번째 과일로 이동 (인덱스: %d)", current_fruit_index_ + 1, priority_list_[current_fruit_index_]);
    sendPathCommand(fruit_positions_, priority_list_);
    move_command_sent = true;
  }

  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "HARVEST_LOOP: 과일 위치 도착");
    current_position_ = fruit_positions_[priority_list_[current_fruit_index_]];
    move_command_sent = false;
    arrival_complete_ = false;
    changeState(MasterState::FOUNDATION_PROCESSING);
  }

  if (elapsed > 60.0) {  // 60초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "HARVEST_LOOP: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleFoundationProcessingState(double elapsed) {
  static bool foundation_started = false;

  if (!foundation_started) {
    RCLCPP_INFO(this->get_logger(), "FOUNDATION: 6D 포즈 추정 시작");
    activateFoundation();
    foundation_started = true;
  }

  if (foundation_complete_) {
    RCLCPP_INFO(this->get_logger(), "FOUNDATION: 6D 포즈 추정 완료");
    foundation_started = false;
    foundation_complete_ = false;
    changeState(MasterState::GRIPPER_OPEN);
  }

  if (elapsed > 30.0) {  // 30초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "FOUNDATION: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleGripperOpenState(double elapsed) {
  static bool gripper_opened = false;

  if (!gripper_opened) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_OPEN: 그리퍼 벌리기");
    sendGripperCommand(true);  // true = open
    gripper_opened = true;
  }

  // 그리퍼 동작 완료 시간 대기 (3초)
  if (elapsed > 3.0) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_OPEN: 그리퍼 벌리기 완료");
    gripper_opened = false;
    changeState(MasterState::MOVE_TO_CUTTING);
  }
}

void MasterNode::handleMoveToCuttingState(double elapsed) {
  static bool cutting_move_sent = false;

  if (!cutting_move_sent) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_CUTTING: 절단점으로 이동");
    sendPathCommand(fruit_positions_, priority_list_);  // 현재 과일 정보 포함
    cutting_move_sent = true;
  }

  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_CUTTING: 절단점 도착");
    current_position_ = cutting_point_;
    cutting_move_sent = false;
    arrival_complete_ = false;
    changeState(MasterState::GRIPPER_CLOSE);
  }

  if (elapsed > 30.0) {  // 30초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "MOVE_TO_CUTTING: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleGripperCloseState(double elapsed) {
  static bool gripper_closed = false;

  if (!gripper_closed) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_CLOSE: 그리퍼 닫기 (수확)");
    sendGripperCommand(false);  // false = close
    gripper_closed = true;
  }

  // 그리퍼 동작 완료 시간 대기 (3초)
  if (elapsed > 3.0) {
    RCLCPP_INFO(this->get_logger(), "GRIPPER_CLOSE: 수확 완료");
    gripper_closed = false;
    changeState(MasterState::MOVE_BACK);
  }
}

void MasterNode::handleMoveBackState(double elapsed) {
  static bool back_move_sent = false;

  if (!back_move_sent) {
    RCLCPP_INFO(this->get_logger(), "MOVE_BACK: 과일 위치로 복귀");
    sendPathCommand(fruit_positions_, priority_list_);
    back_move_sent = true;
  }

  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_BACK: 과일 위치 복귀 완료");
    current_position_ = fruit_positions_[priority_list_[current_fruit_index_]];
    back_move_sent = false;
    arrival_complete_ = false;
    current_fruit_index_++;
    changeState(MasterState::MOVE_TO_NEXT);
  }

  if (elapsed > 30.0) {  // 30초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "MOVE_BACK: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleMoveToNextState(double elapsed) {
  if (current_fruit_index_ >= priority_list_.size()) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT: 모든 과일 처리 완료");
    changeState(MasterState::RETURN_HOME);
    return;
  }

  static bool next_move_sent = false;

  if (!next_move_sent) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT: 다음 과일로 이동");
    sendPathCommand(fruit_positions_, priority_list_);
    next_move_sent = true;
  }

  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "MOVE_TO_NEXT: 다음 과일 위치 도착");
    current_position_ = fruit_positions_[priority_list_[current_fruit_index_]];
    next_move_sent = false;
    arrival_complete_ = false;
    changeState(MasterState::FOUNDATION_PROCESSING);
  }

  if (elapsed > 60.0) {  // 60초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "MOVE_TO_NEXT: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleReturnHomeState(double elapsed) {
  static bool return_sent = false;

  if (!return_sent) {
    RCLCPP_INFO(this->get_logger(), "RETURN_HOME: 시작점으로 복귀");
    sendPathCommand(fruit_positions_, std::vector<int>());  // 우선순위 없이 복귀
    return_sent = true;
  }

  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "RETURN_HOME: 시작점 복귀 완료");
    return_sent = false;
    arrival_complete_ = false;
    changeState(MasterState::SHUTDOWN);
  }

  if (elapsed > 60.0) {  // 60초 타임아웃
    RCLCPP_ERROR(this->get_logger(), "RETURN_HOME: 타임아웃!");
    changeState(MasterState::SHUTDOWN);
  }
}

void MasterNode::handleShutdownState() {
  RCLCPP_INFO(this->get_logger(), "SHUTDOWN: 모든 노드 종료 신호 전송");
  sendShutdownSignal();
  rclcpp::shutdown();
}

// ===== 유틸리티 함수들 =====

void MasterNode::changeState(MasterState new_state) {
  current_state_ = new_state;
  state_start_time_ = this->now();
}

void MasterNode::sendPathCommand(const std::vector<geometry_msgs::msg::Point>& fruit_positions, const std::vector<int>& priority_order) {
  auto msg = vision_msgs::msg::HarvestOrdering();

  // 헤더 설정
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  // 과일 위치들을 DetectedCrop으로 변환
  for (size_t i = 0; i < fruit_positions.size(); ++i) {
    vision_msgs::msg::DetectedCrop crop;
    crop.id = static_cast<uint32_t>(i + 1);  // 1부터 시작하는 ID
    crop.x = fruit_positions[i].x;
    crop.y = fruit_positions[i].y;
    crop.z = fruit_positions[i].z;
    msg.objects.push_back(crop);
  }

  // 총 객체 수 설정
  msg.total_objects = static_cast<uint32_t>(fruit_positions.size());

  // 우선순위 설정 (있는 경우)
  if (!priority_order.empty()) {
    for (int id : priority_order) {
      msg.crop_ids.push_back(static_cast<uint32_t>(id + 1));  // 1부터 시작
    }
  }

  path_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Path command sent: %zu crops, %zu priorities", fruit_positions.size(), priority_order.size());
}

void MasterNode::sendGripperCommand(bool open) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 그리퍼 서비스 클라이언트 선택
  auto client = open ? gripper_open_client_ : gripper_close_client_;
  std::string action = open ? "열기" : "닫기";

  // 서비스 가용성 확인
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "그리퍼 %s 서비스를 사용할 수 없습니다", action.c_str());
    return;
  }

  // 비동기 서비스 호출
  auto future = client->async_send_request(request);

  // 콜백 설정
  auto callback = [this, action](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
    try {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "그리퍼 %s 성공: %s", action.c_str(), response->message.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "그리퍼 %s 실패: %s", action.c_str(), response->message.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "그리퍼 %s 서비스 호출 예외: %s", action.c_str(), e.what());
    }
  };

  // 콜백 등록
  auto future_with_callback = future.then(callback);

  RCLCPP_INFO(this->get_logger(), "그리퍼 %s 명령 전송됨", action.c_str());
}

void MasterNode::activateYolo() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  yolo_pub_->publish(msg);
}

void MasterNode::activateTsp() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  tsp_pub_->publish(msg);
}

void MasterNode::activateFoundation() {
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  foundation_pub_->publish(msg);
}

void MasterNode::sendFruitPositionsToMap() {
  auto msg = std_msgs::msg::Int32MultiArray();
  // 과일 위치 정보를 map_node로 전송 (구현에 따라 수정 필요)
  map_pub_->publish(msg);
}

void MasterNode::sendShutdownSignal() {
  auto msg = std_msgs::msg::String();
  msg.data = "shutdown";
  shutdown_pub_->publish(msg);
}

// ===== 콜백 함수들 =====

void MasterNode::nodeStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data == "all_ready") {
    nodes_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "모든 노드 준비 완료");
  }
}

void MasterNode::movementCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  movement_complete_ = msg->data;
  if (movement_complete_) {
    RCLCPP_INFO(this->get_logger(), "이동 완료 신호 수신");
  }
}

void MasterNode::fruitDetectionCallback(const vision_msgs::msg::DetectedCropArray::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "과일 감지 결과 수신: %d개 과일", msg->total_objects);

  // 과일 위치 저장
  fruit_positions_.clear();
  for (const auto& crop : msg->objects) {
    geometry_msgs::msg::Point point;
    point.x = crop.x;
    point.y = crop.y;
    point.z = crop.z;
    fruit_positions_.push_back(point);

    RCLCPP_INFO(this->get_logger(), "과일 %d: (%.3f, %.3f, %.3f)", crop.id, crop.x, crop.y, crop.z);
  }

  recognition_complete_ = true;
}

void MasterNode::tspCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  priority_list_ = msg->data;
  tsp_complete_ = true;
  RCLCPP_INFO(this->get_logger(), "TSP 결과 수신: %zu개 과일", priority_list_.size());
}

void MasterNode::foundationCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  cutting_point_ = *msg;
  foundation_complete_ = true;
  RCLCPP_INFO(this->get_logger(), "절단점 수신: (%.3f, %.3f, %.3f)", cutting_point_.x, cutting_point_.y, cutting_point_.z);
}

void MasterNode::arrivalCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  arrival_complete_ = msg->data;
  if (arrival_complete_) {
    RCLCPP_INFO(this->get_logger(), "목표 도착 신호 수신");
  }
}

// ===== Main 함수 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}