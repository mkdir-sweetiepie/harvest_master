// master_node.hpp
#ifndef MASTER_NODE_HPP
#define MASTER_NODE_HPP

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/detected_crop_array.hpp"
#include "vision_msgs/msg/harvest_ordering.hpp"

using namespace std::chrono_literals;

enum class MasterState {
  INIT,
  INITIAL_MOVE,
  FRUIT_RECOGNITION,
  TSP_PROCESSING,
  HARVEST_LOOP,
  FOUNDATION_PROCESSING,
  GRIPPER_OPEN,
  MOVE_TO_CUTTING,
  GRIPPER_CLOSE,
  MOVE_BACK,
  MOVE_TO_NEXT,
  RETURN_HOME,
  SHUTDOWN
};

class MasterNode : public rclcpp::Node {
 public:
  MasterNode();
  ~MasterNode() = default;

 private:
  // ===== 상태 및 변수 =====
  MasterState current_state_;
  int current_fruit_index_;
  int total_fruit_count_;
  bool nodes_ready_;
  bool movement_complete_;
  bool recognition_complete_;
  bool tsp_complete_;
  bool foundation_complete_;
  bool arrival_complete_;

  // 위치 정보
  geometry_msgs::msg::Point start_position_;
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point initial_goal_;
  geometry_msgs::msg::Point cutting_point_;
  std::vector<geometry_msgs::msg::Point> fruit_positions_;
  std::vector<int> priority_list_;

  // 타이머
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time state_start_time_;

  // ===== Publishers =====
  rclcpp::Publisher<vision_msgs::msg::HarvestOrdering>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr foundation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yolo_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tsp_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shutdown_pub_;

  // ===== Service Clients =====
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_client_;

  // ===== Subscribers =====
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movement_sub_;
  rclcpp::Subscription<vision_msgs::msg::DetectedCropArray>::SharedPtr fruit_detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tsp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr foundation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrival_sub_;

  // ===== 초기화 함수들 =====
  void initializePublishers();
  void initializeSubscribers();
  void initializeServiceClients();
  void initializeVariables();

  // ===== 메인 상태 머신 =====
  void stateMachineCallback();

  // ===== 상태별 핸들러 함수들 =====
  void handleInitState();
  void handleInitialMoveState(double elapsed);
  void handleFruitRecognitionState(double elapsed);
  void handleTspProcessingState(double elapsed);
  void handleHarvestLoopState(double elapsed);
  void handleFoundationProcessingState(double elapsed);
  void handleGripperOpenState(double elapsed);
  void handleMoveToCuttingState(double elapsed);
  void handleGripperCloseState(double elapsed);
  void handleMoveBackState(double elapsed);
  void handleMoveToNextState(double elapsed);
  void handleReturnHomeState(double elapsed);
  void handleShutdownState();

  // ===== 유틸리티 함수들 =====
  void changeState(MasterState new_state);
  void sendPathCommand(const std::vector<geometry_msgs::msg::Point>& fruit_positions, const std::vector<int>& priority_order = std::vector<int>());
  void sendGripperCommand(bool open);
  void activateYolo();
  void activateTsp();
  void activateFoundation();
  void sendFruitPositionsToMap();
  void sendShutdownSignal();

  // ===== 콜백 함수들 =====
  void nodeStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void movementCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void fruitDetectionCallback(const vision_msgs::msg::DetectedCropArray::SharedPtr msg);
  void tspCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void foundationCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void arrivalCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

#endif  // MASTER_NODE_HPP