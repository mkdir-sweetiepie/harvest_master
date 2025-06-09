/**
 * @file /include/harvest_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date January 2025
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef harvest_master_QNODE_HPP_
#define harvest_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QThread>
#include <QStringListModel>
#include <QPixmap>
#include <QImage>

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  virtual ~QNode();

  // Vision control methods
  void startVisionSystem();
  void stopVisionSystem();
  void captureImage();
  void calibrateCamera();
  
  // Manipulation control methods
  void startHarvest();
  void moveToTarget(double x, double y, double z);
  void controlGripper(bool close);
  void moveToHome();
  void moveToDropZone();
  void emergencyStop();
  void resetArm();

protected:
  void run();

private:
  // ROS2 node
  std::shared_ptr<rclcpp::Node> node;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_control_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  
  // Image processing
  cv::Mat current_image_;
  cv::Mat processed_image_;
  bool vision_active_;
  
  // Callback functions
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void statusCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // Image processing functions
  QPixmap matToQPixmap(const cv::Mat& mat);
  cv::Mat processImage(const cv::Mat& input);
  void detectMelons(const cv::Mat& image);

Q_SIGNALS:
  void rosShutDown();
  void imageReceived(const QPixmap& pixmap);
  void melonDetected(int count, int ripe_count);
  void systemStatusChanged(const QString& status);
  void targetSelected(double x, double y);
};

#endif /* harvest_master_QNODE_HPP_ */