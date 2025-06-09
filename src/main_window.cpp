/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date January 2025
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/harvest_master/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  // Initialize variables
  harvestCount = 0;
  successCount = 0;
  isVisionActive = false;
  isArmReady = true;

  // Setup UI connections
  setupConnections();
  
  // Setup timers
  setupTimers();
  
  // Initialize UI values
  initializeUI();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::setupConnections()
{
  // Vision System Controls
  connect(ui->startVisionBtn, &QPushButton::clicked, this, &MainWindow::onStartVision);
  connect(ui->captureBtn, &QPushButton::clicked, this, &MainWindow::onCapture);
  connect(ui->calibrateBtn, &QPushButton::clicked, this, &MainWindow::onCalibrate);
  
  // Manipulation Controls
  connect(ui->harvestBtn, &QPushButton::clicked, this, &MainWindow::onStartHarvest);
  connect(ui->moveToTargetBtn, &QPushButton::clicked, this, &MainWindow::onMoveToTarget);
  connect(ui->gripBtn, &QPushButton::clicked, this, &MainWindow::onGrip);
  connect(ui->homeBtn, &QPushButton::clicked, this, &MainWindow::onMoveHome);
  connect(ui->dropBtn, &QPushButton::clicked, this, &MainWindow::onMoveToDrop);
  
  // Emergency Controls
  connect(ui->emergencyBtn, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
  connect(ui->resetArmBtn, &QPushButton::clicked, this, &MainWindow::onResetArm);
  connect(ui->resetVisionBtn, &QPushButton::clicked, this, &MainWindow::onResetVision);

  // Menu Actions
  connect(ui->action_StartVision, &QAction::triggered, this, &MainWindow::onStartVision);
  connect(ui->action_Calibrate, &QAction::triggered, this, &MainWindow::onCalibrate);
  connect(ui->action_HomePosition, &QAction::triggered, this, &MainWindow::onMoveHome);
  connect(ui->action_EmergencyStop, &QAction::triggered, this, &MainWindow::onEmergencyStop);
  
  // ROS2 Signal Connections
  connect(qnode, &QNode::imageReceived, this, &MainWindow::onImageReceived);
  connect(qnode, &QNode::melonDetected, this, &MainWindow::onMelonDetected);
  connect(qnode, &QNode::systemStatusChanged, this, &MainWindow::onSystemStatusChanged);
  connect(qnode, &QNode::targetSelected, this, &MainWindow::onTargetSelected);
}

void MainWindow::setupTimers()
{
  // Timer for updating UI data
  updateTimer = new QTimer(this);
  connect(updateTimer, &QTimer::timeout, this, &MainWindow::updateUI);
  updateTimer->start(100); // Update every 100ms
}

void MainWindow::initializeUI()
{
  // Initial status
  ui->armStatusLabel->setText("READY TO HARVEST");
  ui->armPositionLabel->setText("X:0 Y:0 Z:200");
  ui->gripperLabel->setText("OPEN - 준비완료");
  ui->processingLabel->setText("18.5 FPS");
  ui->selectedMelonLabel->setText("대기 중...");
  ui->todayCountLabel->setText("0개");
  ui->avgTimeLabel->setText("--초/개");
  
  // Progress bars
  ui->accuracyProgress->setValue(96);
  ui->successProgress->setValue(100);
  
  // System messages
  appendSystemMessage("[SYSTEM]", "참외 수확 시스템 초기화 완료", "#4a90e2");
}

void MainWindow::updateUI()
{
  // Update harvest statistics
  ui->todayCountLabel->setText(QString::number(harvestCount) + "개");
  
  if (harvestCount > 0) {
    int successRate = (successCount * 100) / harvestCount;
    ui->successProgress->setValue(successRate);
    
    // Calculate average time (example calculation)
    double avgTime = 12.3 + (harvestCount * 0.1); // Simulate getting slower
    ui->avgTimeLabel->setText(QString::number(avgTime, 'f', 1) + "초/개");
  }

  // Update vision status
  if (isVisionActive) {
    ui->startVisionBtn->setText("⏹️ 비전 정지");
    ui->statusLabel->setText("● VISION ACTIVE & ARM READY");
  } else {
    ui->startVisionBtn->setText("🔍 비전 시작");
    ui->statusLabel->setText("● VISION STANDBY & ARM READY");
  }
}

void MainWindow::appendSystemMessage(const QString& type, const QString& message, const QString& color)
{
  QString timestamp = QTime::currentTime().toString("hh:mm:ss");
  QString formattedMsg = QString("<span style='color:%1'>[%2]</span> %3 - %4")
                        .arg(color)
                        .arg(type)
                        .arg(timestamp)
                        .arg(message);
  
  ui->messagesTextEdit->append(formattedMsg);
  
  // Keep only last 50 messages
  QTextDocument* doc = ui->messagesTextEdit->document();
  if (doc->blockCount() > 50) {
    QTextCursor cursor = ui->messagesTextEdit->textCursor();
    cursor.movePosition(QTextCursor::Start);
    cursor.select(QTextCursor::BlockUnderCursor);
    cursor.removeSelectedText();
  }
}

// Vision System Slots
void MainWindow::onStartVision()
{
  if (!isVisionActive) {
    isVisionActive = true;
    qnode->startVisionSystem(); // 실제 ROS2 호출
    appendSystemMessage("VISION", "비전 시스템 시작", "#4a90e2");
  } else {
    isVisionActive = false;
    qnode->stopVisionSystem(); // 실제 ROS2 호출
    appendSystemMessage("VISION", "비전 시스템 정지", "#f39c12");
  }
}

void MainWindow::onCapture()
{
  qnode->captureImage(); // 실제 ROS2 호출
  appendSystemMessage("VISION", "이미지 캡처 완료", "#4a90e2");
}

void MainWindow::onCalibrate()
{
  appendSystemMessage("VISION", "카메라 캘리브레이션 시작", "#f39c12");
  ui->calibrateBtn->setEnabled(false);
  
  qnode->calibrateCamera(); // 실제 ROS2 호출
  
  // Simulate calibration process
  QTimer::singleShot(3000, [this]() {
    ui->calibrateBtn->setEnabled(true);
    appendSystemMessage("VISION", "캘리브레이션 완료", "#2ecc71");
  });
}

// Manipulation System Slots
void MainWindow::onStartHarvest()
{
  if (!isArmReady) {
    appendSystemMessage("ERROR", "로봇 암이 준비되지 않음", "#e74c3c");
    return;
  }
  
  if (!isVisionActive) {
    appendSystemMessage("ERROR", "비전 시스템이 활성화되지 않음", "#e74c3c");
    return;
  }
  
  isArmReady = false;
  ui->harvestBtn->setEnabled(false);
  ui->armStatusLabel->setText("HARVESTING...");
  
  qnode->startHarvest(); // 실제 ROS2 호출
  appendSystemMessage("ARM", "참외 수확 시작", "#27ae60");
  
  // Simulate harvest sequence
  QTimer::singleShot(5000, [this]() {
    harvestCount++;
    successCount++; // For demo, assume always success
    isArmReady = true;
    ui->harvestBtn->setEnabled(true);
    ui->armStatusLabel->setText("READY TO HARVEST");
    
    appendSystemMessage("SUCCESS", QString("참외 #%1 수확 완료").arg(harvestCount), "#2ecc71");
  });
}

void MainWindow::onMoveToTarget()
{
  ui->armStatusLabel->setText("MOVING TO TARGET...");
  appendSystemMessage("ARM", "타겟 위치로 이동 중", "#4a90e2");
  
  // Get target coordinates from vision system (example coordinates)
  qnode->moveToTarget(245, 180, 150); // 실제 ROS2 호출
  
  QTimer::singleShot(2000, [this]() {
    ui->armStatusLabel->setText("AT TARGET POSITION");
    appendSystemMessage("ARM", "타겟 위치 도달", "#2ecc71");
  });
}

void MainWindow::onGrip()
{
  QString currentGripperState = ui->gripperLabel->text();
  
  if (currentGripperState.contains("OPEN")) {
    ui->gripperLabel->setText("CLOSED - 참외 그립");
    appendSystemMessage("ARM", "그리퍼 닫음 - 참외 파지", "#27ae60");
    ui->gripBtn->setText("👐 릴리즈");
    qnode->controlGripper(true); // 실제 ROS2 호출
  } else {
    ui->gripperLabel->setText("OPEN - 준비완료");
    appendSystemMessage("ARM", "그리퍼 열림", "#4a90e2");
    ui->gripBtn->setText("✋ 그립");
    qnode->controlGripper(false); // 실제 ROS2 호출
  }
}

void MainWindow::onMoveHome()
{
  ui->armStatusLabel->setText("MOVING TO HOME...");
  ui->armPositionLabel->setText("X:0 Y:0 Z:200");
  appendSystemMessage("ARM", "홈 위치로 이동", "#4a90e2");
  
  qnode->moveToHome(); // 실제 ROS2 호출
  
  QTimer::singleShot(3000, [this]() {
    ui->armStatusLabel->setText("AT HOME POSITION");
    ui->gripperLabel->setText("OPEN - 준비완료");
    isArmReady = true;
    appendSystemMessage("ARM", "홈 위치 도달", "#2ecc71");
  });
}

void MainWindow::onMoveToDrop()
{
  ui->armStatusLabel->setText("MOVING TO DROP ZONE...");
  ui->armPositionLabel->setText("X:-100 Y:0 Z:150");
  appendSystemMessage("ARM", "수확통으로 이동", "#4a90e2");
  
  qnode->moveToDropZone(); // 실제 ROS2 호출
  
  QTimer::singleShot(2000, [this]() {
    ui->armStatusLabel->setText("AT DROP ZONE");
    appendSystemMessage("ARM", "수확통 위치 도달", "#2ecc71");
  });
}

// Emergency Controls
void MainWindow::onEmergencyStop()
{
  isArmReady = false;
  isVisionActive = false;
  
  qnode->emergencyStop(); // 실제 ROS2 호출
  
  ui->armStatusLabel->setText("🚨 EMERGENCY STOP");
  ui->armStatusLabel->setStyleSheet("color: #e74c3c; font-weight: bold;");
  ui->statusLabel->setText("🚨 EMERGENCY MODE");
  ui->statusLabel->setStyleSheet("color: #e74c3c; font-weight: bold;");
  
  // Disable all control buttons
  ui->harvestBtn->setEnabled(false);
  ui->moveToTargetBtn->setEnabled(false);
  ui->gripBtn->setEnabled(false);
  ui->homeBtn->setEnabled(false);
  ui->dropBtn->setEnabled(false);
  ui->startVisionBtn->setEnabled(false);
  
  appendSystemMessage("EMERGENCY", "비상 정지 활성화", "#e74c3c");
}

void MainWindow::onResetArm()
{
  ui->armStatusLabel->setText("RESETTING ARM...");
  ui->armStatusLabel->setStyleSheet(""); // Reset style
  
  qnode->resetArm(); // 실제 ROS2 호출
  appendSystemMessage("ARM", "로봇 암 리셋 중", "#f39c12");
  
  QTimer::singleShot(5000, [this]() {
    isArmReady = true;
    ui->armStatusLabel->setText("READY TO HARVEST");
    ui->armPositionLabel->setText("X:0 Y:0 Z:200");
    ui->gripperLabel->setText("OPEN - 준비완료");
    
    // Re-enable arm control buttons
    ui->harvestBtn->setEnabled(true);
    ui->moveToTargetBtn->setEnabled(true);
    ui->gripBtn->setEnabled(true);
    ui->homeBtn->setEnabled(true);
    ui->dropBtn->setEnabled(true);
    
    ui->statusLabel->setText("● VISION & MANIPULATION READY");
    ui->statusLabel->setStyleSheet(""); // Reset style
    
    appendSystemMessage("ARM", "로봇 암 리셋 완료", "#2ecc71");
  });
}

void MainWindow::onResetVision()
{
  ui->selectedMelonLabel->setText("리셋 중...");
  appendSystemMessage("VISION", "비전 시스템 리셋 중", "#f39c12");
  
  // Stop vision first, then restart
  qnode->stopVisionSystem();
  
  QTimer::singleShot(3000, [this]() {
    isVisionActive = false;
    ui->selectedMelonLabel->setText("대기 중...");
    ui->startVisionBtn->setEnabled(true);
    ui->calibrateBtn->setEnabled(true);
    
    // Clear camera feed
    ui->cameraFeed->clear();
    ui->cameraFeed->setText("📷 VISION SYSTEM STANDBY\n\n비전 시스템을 시작하세요");
    
    ui->statusLabel->setText("● VISION & MANIPULATION READY");
    ui->statusLabel->setStyleSheet(""); // Reset style
    
    appendSystemMessage("VISION", "비전 시스템 리셋 완료", "#2ecc71");
  });
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  if (updateTimer) {
    updateTimer->stop();
  }
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}

/*****************************************************************************
** ROS2 Signal Slots
*****************************************************************************/

void MainWindow::onImageReceived(const QPixmap& pixmap)
{
  if (!pixmap.isNull()) {
    // Scale image to fit the camera feed label while maintaining aspect ratio
    QPixmap scaledPixmap = pixmap.scaled(ui->cameraFeed->size(), 
                                        Qt::KeepAspectRatio, 
                                        Qt::SmoothTransformation);
    ui->cameraFeed->setPixmap(scaledPixmap);
    ui->cameraFeed->setText(""); // Remove text when showing image
  }
}

void MainWindow::onMelonDetected(int count, int ripe_count)
{
  // Update camera feed info text overlay (could be overlaid on image)
  QString detectionInfo = QString("📷 VISION ACTIVE\n\n🍈 참외 감지됨: %1개\n🟢 수확 가능: %2개\n🔴 미성숙: %3개\n⚠️ 장애물: 없음")
                         .arg(count)
                         .arg(ripe_count)
                         .arg(count - ripe_count);
  
  // Update analysis results
  ui->selectedMelonLabel->setText(QString("감지된 참외: %1개 | 수확 가능: %2개").arg(count).arg(ripe_count));
  
  // Log detection results
  appendSystemMessage("VISION", QString("참외 %1개 감지 (수확가능: %2개)").arg(count).arg(ripe_count), "#4a90e2");
}

void MainWindow::onSystemStatusChanged(const QString& status)
{
  // Update system status based on ROS2 messages
  ui->statusLabel->setText("● " + status);
  appendSystemMessage("SYSTEM", status, "#4a90e2");
}

void MainWindow::onTargetSelected(double x, double y)
{
  // Update target information
  ui->selectedMelonLabel->setText(QString("타겟 선택됨: (%.0f, %.0f)").arg(x).arg(y));
  ui->armPositionLabel->setText(QString("Target: X:%.0f Y:%.0f Z:150").arg(x).arg(y));
  
  appendSystemMessage("VISION", QString("새로운 타겟 선택: (%.0f, %.0f)").arg(x).arg(y), "#27ae60");
}