/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2025
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/harvest_master/qnode.hpp"

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode() : vision_active_(false)
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("harvest_master");
  
  // Initialize subscribers
  image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", 10,
    std::bind(&QNode::imageCallback, this, std::placeholders::_1));
    
  status_subscriber_ = node->create_subscription<std_msgs::msg::String>(
    "/harvest_master/status", 10,
    std::bind(&QNode::statusCallback, this, std::placeholders::_1));
  
  // Initialize publishers
  vision_control_publisher_ = node->create_publisher<std_msgs::msg::Bool>(
    "/harvest_master/vision_control", 10);
    
  target_publisher_ = node->create_publisher<geometry_msgs::msg::Point>(
    "/harvest_master/target_position", 10);
    
  command_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/harvest_master/command", 10);
  
  RCLCPP_INFO(node->get_logger(), "Harvest Master Node initialized");
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

/*****************************************************************************
** Vision System Methods
*****************************************************************************/

void QNode::startVisionSystem()
{
  vision_active_ = true;
  
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  vision_control_publisher_->publish(msg);
  
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "VISION_STARTED";
  command_publisher_->publish(status_msg);
  
  RCLCPP_INFO(node->get_logger(), "Vision system started");
}

void QNode::stopVisionSystem()
{
  vision_active_ = false;
  
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  vision_control_publisher_->publish(msg);
  
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "VISION_STOPPED";
  command_publisher_->publish(status_msg);
  
  RCLCPP_INFO(node->get_logger(), "Vision system stopped");
}

void QNode::captureImage()
{
  if (!current_image_.empty())
  {
    std::string filename = "/tmp/harvest_capture_" + 
                          std::to_string(std::time(nullptr)) + ".jpg";
    cv::imwrite(filename, current_image_);
    
    auto msg = std_msgs::msg::String();
    msg.data = "IMAGE_CAPTURED:" + filename;
    command_publisher_->publish(msg);
    
    RCLCPP_INFO(node->get_logger(), "Image captured: %s", filename.c_str());
  }
}

void QNode::calibrateCamera()
{
  auto msg = std_msgs::msg::String();
  msg.data = "CALIBRATE_CAMERA";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Camera calibration requested");
}

/*****************************************************************************
** Manipulation Methods
*****************************************************************************/

void QNode::startHarvest()
{
  auto msg = std_msgs::msg::String();
  msg.data = "START_HARVEST";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Harvest sequence started");
}

void QNode::moveToTarget(double x, double y, double z)
{
  auto msg = geometry_msgs::msg::Point();
  msg.x = x;
  msg.y = y;
  msg.z = z;
  target_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Moving to target: (%.2f, %.2f, %.2f)", x, y, z);
}

void QNode::controlGripper(bool close)
{
  auto msg = std_msgs::msg::String();
  msg.data = close ? "GRIPPER_CLOSE" : "GRIPPER_OPEN";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Gripper command: %s", msg.data.c_str());
}

void QNode::moveToHome()
{
  auto msg = std_msgs::msg::String();
  msg.data = "MOVE_HOME";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Moving to home position");
}

void QNode::moveToDropZone()
{
  auto msg = std_msgs::msg::String();
  msg.data = "MOVE_DROP_ZONE";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Moving to drop zone");
}

void QNode::emergencyStop()
{
  auto msg = std_msgs::msg::String();
  msg.data = "EMERGENCY_STOP";
  command_publisher_->publish(msg);
  
  RCLCPP_ERROR(node->get_logger(), "EMERGENCY STOP ACTIVATED");
}

void QNode::resetArm()
{
  auto msg = std_msgs::msg::String();
  msg.data = "RESET_ARM";
  command_publisher_->publish(msg);
  
  RCLCPP_INFO(node->get_logger(), "Arm reset requested");
}

/*****************************************************************************
** Callback Functions
*****************************************************************************/

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert ROS image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    current_image_ = cv_ptr->image.clone();
    
    if (vision_active_)
    {
      // Process the image for melon detection
      processed_image_ = processImage(current_image_);
      
      // Detect melons
      detectMelons(processed_image_);
      
      // Convert processed image to QPixmap and emit signal
      QPixmap pixmap = matToQPixmap(processed_image_);
      Q_EMIT imageReceived(pixmap);
    }
    else
    {
      // Just display raw image
      QPixmap pixmap = matToQPixmap(current_image_);
      Q_EMIT imageReceived(pixmap);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void QNode::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  Q_EMIT systemStatusChanged(QString::fromStdString(msg->data));
}

/*****************************************************************************
** Image Processing Functions
*****************************************************************************/

QPixmap QNode::matToQPixmap(const cv::Mat& mat)
{
  if (mat.empty()) {
    return QPixmap();
  }
  
  QImage qimg;
  
  if (mat.channels() == 3) {
    // BGR to RGB conversion
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    qimg = QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
  } else if (mat.channels() == 1) {
    qimg = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
  }
  
  return QPixmap::fromImage(qimg);
}

cv::Mat QNode::processImage(const cv::Mat& input)
{
  cv::Mat processed = input.clone();
  
  if (!vision_active_) {
    return processed;
  }
  
  // Apply image processing for better melon detection
  cv::Mat hsv, mask, result;
  cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
  
  // Define color range for yellow/orange melons (참외)
  cv::Scalar lower_yellow(15, 50, 50);   // Lower HSV threshold
  cv::Scalar upper_yellow(35, 255, 255); // Upper HSV threshold
  
  // Create mask for yellow/orange colors
  cv::inRange(hsv, lower_yellow, upper_yellow, mask);
  
  // Apply morphological operations to clean up the mask
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  
  // Apply mask to original image
  input.copyTo(result, mask);
  
  // Combine original and masked image for better visualization
  cv::addWeighted(input, 0.7, result, 0.3, 0, processed);
  
  return processed;
}

void QNode::detectMelons(const cv::Mat& image)
{
  cv::Mat hsv, mask;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  
  // Define color range for ripe melons (yellow/orange)
  cv::Scalar lower_ripe(15, 50, 50);
  cv::Scalar upper_ripe(35, 255, 255);
  cv::inRange(hsv, lower_ripe, upper_ripe, mask);
  
  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  int total_melons = 0;
  int ripe_melons = 0;
  double target_x = -1, target_y = -1;
  double max_area = 0;
  
  for (const auto& contour : contours)
  {
    double area = cv::contourArea(contour);
    
    // Filter by minimum area (adjust based on your camera setup)
    if (area > 1000) // Minimum area threshold
    {
      total_melons++;
      
      // Calculate centroid
      cv::Moments moments = cv::moments(contour);
      if (moments.m00 > 0)
      {
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;
        
        // Draw bounding box and label
        cv::Rect bbox = cv::boundingRect(contour);
        cv::rectangle(processed_image_, bbox, cv::Scalar(0, 255, 0), 2);
        
        // Check if this melon is "ripe" based on color intensity
        cv::Mat roi = hsv(bbox);
        cv::Scalar mean_color = cv::mean(roi, mask(bbox));
        
        if (mean_color[1] > 100) // Saturation threshold for ripeness
        {
          ripe_melons++;
          cv::putText(processed_image_, "RIPE", 
                     cv::Point(bbox.x, bbox.y - 10),
                     cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
          
          // Select largest ripe melon as target
          if (area > max_area)
          {
            max_area = area;
            target_x = cx;
            target_y = cy;
          }
        }
        else
        {
          cv::putText(processed_image_, "UNRIPE", 
                     cv::Point(bbox.x, bbox.y - 10),
                     cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
        
        // Draw center point
        cv::circle(processed_image_, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
      }
    }
  }
  
  // Draw crosshair for selected target
  if (target_x > 0 && target_y > 0)
  {
    cv::line(processed_image_, 
             cv::Point(target_x - 20, target_y), 
             cv::Point(target_x + 20, target_y), 
             cv::Scalar(0, 0, 255), 3);
    cv::line(processed_image_, 
             cv::Point(target_x, target_y - 20), 
             cv::Point(target_x, target_y + 20), 
             cv::Scalar(0, 0, 255), 3);
    
    Q_EMIT targetSelected(target_x, target_y);
  }
  
  // Add detection info overlay
  std::string info = "Total: " + std::to_string(total_melons) + 
                    " | Ripe: " + std::to_string(ripe_melons);
  cv::putText(processed_image_, info, cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
  
  Q_EMIT melonDetected(total_melons, ripe_melons);
}