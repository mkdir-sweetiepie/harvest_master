// include/harvest_master/qnode.hpp
#ifndef HARVEST_MASTER_QNODE_HPP_
#define HARVEST_MASTER_QNODE_HPP_

#include <QString>
#include <QStringListModel>
#include <QThread>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  virtual ~QNode();

  // 시스템 제어 메서드
  void startHarvestSequence();
  void stopHarvestSequence();
  void emergencyStop();
  void resetSystem();

  // 개별 모듈 제어
  void triggerVisionDetection();
  void moveToPosition(double x, double y, double z, double rx, double ry, double rz);
  void setJointVelocities(const std::vector<double>& velocities);
  void requestPathPlanning(const geometry_msgs::msg::Pose& target);
  void activateCuttingTool();

  // 상태 조회
  QString getCurrentState() const { return current_state_; }
  int getDetectedCropsCount() const { return detected_crops_count_; }
  int getCurrentTargetIndex() const { return current_target_index_; }
  QString getLastError() const { return last_error_; }

 protected:
  void run() override;

 Q_SIGNALS:
  void rosShutDown();

  // GUI 업데이트 시그널
  void stateChanged(QString state);
  void cropsDetected(int count);
  void harvestOrderUpdated(QStringList order);
  void pathPlanningComplete();
  void robotStatusUpdated(QString status);
  void errorOccurred(QString error);
  void progressUpdated(int current, int total);
  void logMessage(QString message);

  // 비전 시스템 시그널
  void visionSystemReady();
  void detectionComplete(int ripe_count, int unripe_count);
  void foundationPoseComplete();

  // 모터 제어 시스템 시그널
  void motorSystemReady();
  void positionReached();
  void cuttingComplete();

  // 경로 계획 시스템 시그널
  void pathPlanningReady();
  void optimalPathCalculated();

 private:
  // ROS 2 노드 및 통신
  std::shared_ptr<rclcpp::Node> node_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr path_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cutting_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr crop_detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr harvest_order_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr planned_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr foundation_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_status_sub_;

  // 콜백 함수들
  void cropDetectionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void harvestOrderCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void plannedPathCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void foundationPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void robotStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void visionStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void motorStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void planningStatusCallback(const std_msgs::msg::String::SharedPtr msg);

  // 상태 변수들
  QString current_state_;
  int detected_crops_count_;
  int current_target_index_;
  int total_targets_;
  QString last_error_;
  bool system_ready_;
  bool vision_ready_;
  bool motor_ready_;
  bool planning_ready_;

  // 데이터 저장
  std::vector<geometry_msgs::msg::Pose> detected_crops_;
  std::vector<int> harvest_order_;
  geometry_msgs::msg::Pose current_target_pose_;
};

#endif  // HARVEST_MASTER_QNODE_HPP_