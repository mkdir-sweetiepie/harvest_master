// master_node.hpp
#ifndef MASTER_NODE_HPP
#define MASTER_NODE_HPP

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/crop_pose.hpp"
#include "vision_msgs/msg/harvest_ordering.hpp"

using namespace std::chrono_literals;

enum class MasterState {
  INIT,                   // 1. 모든 노드 준비 대기
  INITIAL_MOVE,           // 2. 초기 위치로 이동
  FRUIT_RECOGNITION,      // 3. 과일 인식 수행
  TSP_PROCESSING,         // 4. 최적 경로 계산
  HARVEST_LOOP,           // 5. 과일별 수확 루프 시작
  FOUNDATION_PROCESSING,  // 6. 6D 포즈 추정
  GRIPPER_OPEN,           // 7. 그리퍼 열기
  MOVE_TO_CUTTING,        // 8. 절단점으로 이동
  GRIPPER_CLOSE,          // 9. 그리퍼 닫기(수확)
  MOVE_BACK,              // 10. 과일 위치로 복귀
  MOVE_TO_NEXT,           // 11. 다음 과일로 이동
  RETURN_HOME,            // 12. 시작점으로 복귀
  ERROR_STATE,            // 13. 에러 상태
  SHUTDOWN                // 14. 시스템 종료
};

class MasterNode : public rclcpp::Node {
 public:
  MasterNode();
  ~MasterNode();

 private:
  // ===== 상태 및 변수 =====
  std::atomic<MasterState> current_state_;
  std::atomic<int> current_fruit_index_;
  int total_fruit_count_;

  // 스레드 안전성을 위한 뮤텍스
  std::mutex state_mutex_;
  std::mutex data_mutex_;

  // 상태별 플래그들
  bool initial_command_sent_;
  bool recognition_started_;
  bool move_command_sent_;
  bool foundation_started_;
  bool gripper_opened_;
  bool cutting_move_sent_;
  bool gripper_closed_;
  bool back_move_sent_;
  bool next_move_sent_;
  bool return_sent_;
  int error_count_;

  // 원래 플래그들
  bool nodes_ready_;
  bool yolo_ready_;
  bool tsp_ready_;
  bool foundation_ready_;
  bool path_ready_;
  std::atomic<bool> movement_complete_;
  std::atomic<bool> tsp_complete_;
  std::atomic<bool> foundation_complete_;
  std::atomic<bool> gripper_open_complete_;
  std::atomic<bool> gripper_close_complete_;

  // 위치 정보
  bool start_;
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point initial_goal_;
  geometry_msgs::msg::Point cutting_point_;
  std::vector<geometry_msgs::msg::Point> fruit_positions_;
  std::vector<int> priority_list_;

  // 타이머
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time state_start_time_;

  // ===== Publishers =====
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pub_;
  rclcpp::Publisher<vision_msgs::msg::HarvestOrdering>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr foundation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yolo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shutdown_pub_;

  // ===== Service Clients =====
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_client_;

  // ===== Subscribers =====
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movement_sub_;
  rclcpp::Subscription<vision_msgs::msg::HarvestOrdering>::SharedPtr tsp_sub_;
  rclcpp::Subscription<vision_msgs::msg::CropPose>::SharedPtr foundation_sub_;

  // ===== 초기화 함수들 =====
  void initializePublishers();
  void initializeSubscribers();
  void initializeServiceClients();
  void initializeVariables();

  // ===== 메인 상태 머신 =====
  void stateMachineCallback();

  // ===== 상태별 핸들러 함수들 =====
  void handleInitState();
  void handleInitialMoveState();
  void handleFruitRecognitionState();
  void handleTspProcessingState();
  void handleHarvestLoopState();
  void handleFoundationProcessingState();
  void handleGripperOpenState();
  void handleMoveToCuttingState();
  void handleGripperCloseState();
  void handleMoveBackState();
  void handleMoveToNextState();
  void handleReturnHomeState();
  void handleErrorState();
  void handleShutdownState();

  // ===== 유틸리티 함수들 =====
  void changeState(MasterState new_state);
  void resetStateFlags();

  // 로봇 명령 전송 (용도별 구분)
  void sendInitialCommand();
  void sendTspCommand();
  void sendFoundationCommand();
  void sendReturnHomeCommand();
  void sendGripperCommand(bool open);
  void activateYolo(bool activate = true);        // 🔹 매개변수 추가
  void activateFoundation(bool activate = true);  // 🔹 매개변수 추가
  void sendShutdownSignal();

  // ===== 개별 전송 함수들 =====
  void sendStartCommand(bool start_signal);
  void sendGoalCommand(const std::vector<double>& goal_array);
  void sendPathCommand(const vision_msgs::msg::HarvestOrdering& harvest_order);

  // ===== 콜백 함수들 =====
  void movementCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void tspCallback(const vision_msgs::msg::HarvestOrdering::SharedPtr msg);
  void foundationCallback(const vision_msgs::msg::CropPose::SharedPtr msg);
};

#endif  // MASTER_NODE_HPP