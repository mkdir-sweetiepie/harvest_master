// src/main_window.cpp
#include "../include/harvest_master/main_window.hpp"

#include <QFile>
#include <QIODevice>
#include <cmath>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign), isHarvesting_(false), currentState_("DISCONNECTED"), detectedCropsCount_(0), currentProgress_(0), totalProgress_(0) {
  ui->setupUi(this);

  // 창 설정
  setWindowTitle("BARAM 농작물 수확 로봇 관제 시스템");
  setMinimumSize(1200, 800);

  // QNode 초기화
  qnode_ = new QNode();

  // UI 설정
  setupUI();
  setupConnections();

  // 상태 업데이트 타이머
  statusUpdateTimer_ = new QTimer(this);
  connect(statusUpdateTimer_, &QTimer::timeout, this, &MainWindow::updateSystemStatus);
  statusUpdateTimer_->start(1000);  // 1초마다 업데이트

  logMessage("시스템 초기화 완료");
}

MainWindow::~MainWindow() {
  delete ui;
  delete qnode_;
}

void MainWindow::setupUI() {
  // 중앙 위젯 설정
  QWidget* centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  // 메인 레이아웃
  QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

  // 상단 상태 표시 영역
  statusGroup_ = new QGroupBox("시스템 상태", this);
  QGridLayout* statusLayout = new QGridLayout(statusGroup_);

  statusLayout->addWidget(new QLabel("전체 상태:"), 0, 0);
  stateLabel_ = new QLabel("DISCONNECTED");
  stateLabel_->setStyleSheet("QLabel { background-color: #ffcccc; padding: 5px; border-radius: 3px; }");
  statusLayout->addWidget(stateLabel_, 0, 1);

  statusLayout->addWidget(new QLabel("비전 시스템:"), 1, 0);
  visionStatusLabel_ = new QLabel("대기중");
  visionStatusLabel_->setStyleSheet("QLabel { background-color: #ffffcc; padding: 5px; border-radius: 3px; }");
  statusLayout->addWidget(visionStatusLabel_, 1, 1);

  statusLayout->addWidget(new QLabel("모터 시스템:"), 1, 2);
  motorStatusLabel_ = new QLabel("대기중");
  motorStatusLabel_->setStyleSheet("QLabel { background-color: #ffffcc; padding: 5px; border-radius: 3px; }");
  statusLayout->addWidget(motorStatusLabel_, 1, 3);

  statusLayout->addWidget(new QLabel("경로 계획:"), 2, 0);
  planningStatusLabel_ = new QLabel("대기중");
  planningStatusLabel_->setStyleSheet("QLabel { background-color: #ffffcc; padding: 5px; border-radius: 3px; }");
  statusLayout->addWidget(planningStatusLabel_, 2, 1);

  statusLayout->addWidget(new QLabel("로봇 상태:"), 2, 2);
  robotStatusLabel_ = new QLabel("대기중");
  robotStatusLabel_->setStyleSheet("QLabel { background-color: #ffffcc; padding: 5px; border-radius: 3px; }");
  statusLayout->addWidget(robotStatusLabel_, 2, 3);

  mainLayout->addWidget(statusGroup_);

  // 탭 위젯 생성
  mainTabWidget_ = new QTabWidget(this);
  mainLayout->addWidget(mainTabWidget_);

  setupControlTab();
  setupVisionTab();
  setupMotorTab();
  setupPlanningTab();
  setupLogTab();
}

void MainWindow::setupControlTab() {
  controlTab_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(controlTab_);

  // 메인 제어 버튼들
  QGroupBox* controlGroup = new QGroupBox("메인 제어");
  QGridLayout* controlLayout = new QGridLayout(controlGroup);

  startButton_ = new QPushButton("수확 시작");
  startButton_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 14px; padding: 10px; }");
  startButton_->setMinimumHeight(50);
  controlLayout->addWidget(startButton_, 0, 0);

  stopButton_ = new QPushButton("수확 중지");
  stopButton_->setStyleSheet("QPushButton { background-color: #FFC107; color: black; font-size: 14px; padding: 10px; }");
  stopButton_->setMinimumHeight(50);
  stopButton_->setEnabled(false);
  controlLayout->addWidget(stopButton_, 0, 1);

  emergencyButton_ = new QPushButton("긴급 정지");
  emergencyButton_->setStyleSheet("QPushButton { background-color: #F44336; color: white; font-size: 14px; padding: 10px; }");
  emergencyButton_->setMinimumHeight(50);
  controlLayout->addWidget(emergencyButton_, 1, 0);

  resetButton_ = new QPushButton("시스템 초기화");
  resetButton_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-size: 14px; padding: 10px; }");
  resetButton_->setMinimumHeight(50);
  controlLayout->addWidget(resetButton_, 1, 1);

  layout->addWidget(controlGroup);

  // 진행 상황 표시
  QGroupBox* progressGroup = new QGroupBox("진행 상황");
  QVBoxLayout* progressLayout = new QVBoxLayout(progressGroup);

  progressLabel_ = new QLabel("대기 중");
  progressLayout->addWidget(progressLabel_);

  progressBar_ = new QProgressBar();
  progressBar_->setMinimum(0);
  progressBar_->setMaximum(100);
  progressBar_->setValue(0);
  progressLayout->addWidget(progressBar_);

  layout->addWidget(progressGroup);

  mainTabWidget_->addTab(controlTab_, "메인 제어");
}

void MainWindow::setupVisionTab() {
  visionTab_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(visionTab_);

  // 비전 제어
  QGroupBox* visionGroup = new QGroupBox("비전 시스템 제어");
  QGridLayout* visionLayout = new QGridLayout(visionGroup);

  triggerDetectionButton_ = new QPushButton("수동 감지 시작");
  visionLayout->addWidget(triggerDetectionButton_, 0, 0);

  calibrateCameraButton_ = new QPushButton("카메라 캘리브레이션");
  visionLayout->addWidget(calibrateCameraButton_, 0, 1);

  layout->addWidget(visionGroup);

  // 감지된 참외 정보
  QGroupBox* detectionGroup = new QGroupBox("감지 결과");
  QVBoxLayout* detectionLayout = new QVBoxLayout(detectionGroup);

  detectedCropsLabel_ = new QLabel("감지된 참외: 0개");
  detectedCropsLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; }");
  detectionLayout->addWidget(detectedCropsLabel_);

  cropsListWidget_ = new QListWidget();
  detectionLayout->addWidget(cropsListWidget_);

  layout->addWidget(detectionGroup);

  mainTabWidget_->addTab(visionTab_, "비전 시스템");
}

void MainWindow::setupMotorTab() {
  motorTab_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(motorTab_);

  // 수동 제어
  manualControlGroup_ = new QGroupBox("수동 위치 제어");
  QGridLayout* manualLayout = new QGridLayout(manualControlGroup_);

  // 위치 입력
  manualLayout->addWidget(new QLabel("X (mm):"), 0, 0);
  xSpinBox_ = new QDoubleSpinBox();
  xSpinBox_->setRange(-1000, 1000);
  xSpinBox_->setSingleStep(1.0);
  xSpinBox_->setSuffix(" mm");
  manualLayout->addWidget(xSpinBox_, 0, 1);

  manualLayout->addWidget(new QLabel("Y (mm):"), 0, 2);
  ySpinBox_ = new QDoubleSpinBox();
  ySpinBox_->setRange(-1000, 1000);
  ySpinBox_->setSingleStep(1.0);
  ySpinBox_->setSuffix(" mm");
  manualLayout->addWidget(ySpinBox_, 0, 3);

  manualLayout->addWidget(new QLabel("Z (mm):"), 1, 0);
  zSpinBox_ = new QDoubleSpinBox();
  zSpinBox_->setRange(0, 1500);
  zSpinBox_->setSingleStep(1.0);
  zSpinBox_->setSuffix(" mm");
  manualLayout->addWidget(zSpinBox_, 1, 1);

  // 회전 입력
  manualLayout->addWidget(new QLabel("RX (deg):"), 2, 0);
  rxSpinBox_ = new QDoubleSpinBox();
  rxSpinBox_->setRange(-180, 180);
  rxSpinBox_->setSingleStep(1.0);
  rxSpinBox_->setSuffix("°");
  manualLayout->addWidget(rxSpinBox_, 2, 1);

  manualLayout->addWidget(new QLabel("RY (deg):"), 2, 2);
  rySpinBox_ = new QDoubleSpinBox();
  rySpinBox_->setRange(-180, 180);
  rySpinBox_->setSingleStep(1.0);
  rySpinBox_->setSuffix("°");
  manualLayout->addWidget(rySpinBox_, 2, 3);

  manualLayout->addWidget(new QLabel("RZ (deg):"), 3, 0);
  rzSpinBox_ = new QDoubleSpinBox();
  rzSpinBox_->setRange(-180, 180);
  rzSpinBox_->setSingleStep(1.0);
  rzSpinBox_->setSuffix("°");
  manualLayout->addWidget(rzSpinBox_, 3, 1);

  moveButton_ = new QPushButton("위치로 이동");
  moveButton_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 10px; }");
  manualLayout->addWidget(moveButton_, 4, 0, 1, 4);

  layout->addWidget(manualControlGroup_);

  // 테스트 기능
  QGroupBox* testGroup = new QGroupBox("테스트 기능");
  QGridLayout* testLayout = new QGridLayout(testGroup);

  testCuttingButton_ = new QPushButton("절단 도구 테스트");
  testCuttingButton_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; padding: 10px; }");
  testLayout->addWidget(testCuttingButton_, 0, 0);

  QPushButton* homeButton = new QPushButton("홈 포지션으로 이동");
  homeButton->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 10px; }");
  testLayout->addWidget(homeButton, 0, 1);

  layout->addWidget(testGroup);

  mainTabWidget_->addTab(motorTab_, "모터 제어");
}

void MainWindow::setupPlanningTab() {
  planningTab_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(planningTab_);

  // 수확 순서 표시
  QGroupBox* orderGroup = new QGroupBox("수확 순서 (TSP 최적화)");
  QVBoxLayout* orderLayout = new QVBoxLayout(orderGroup);

  harvestOrderWidget_ = new QListWidget();
  orderLayout->addWidget(harvestOrderWidget_);

  recalculatePathButton_ = new QPushButton("경로 재계산");
  recalculatePathButton_->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; padding: 10px; }");
  orderLayout->addWidget(recalculatePathButton_);

  layout->addWidget(orderGroup);

  // 현재 목표 정보
  QGroupBox* targetGroup = new QGroupBox("현재 목표");
  QVBoxLayout* targetLayout = new QVBoxLayout(targetGroup);

  currentTargetLabel_ = new QLabel("목표 없음");
  currentTargetLabel_->setStyleSheet("QLabel { font-size: 16px; font-weight: bold; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }");
  targetLayout->addWidget(currentTargetLabel_);

  layout->addWidget(targetGroup);

  mainTabWidget_->addTab(planningTab_, "경로 계획");
}

void MainWindow::setupLogTab() {
  logTab_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(logTab_);

  // 로그 표시
  QGroupBox* logGroup = new QGroupBox("시스템 로그");
  QVBoxLayout* logLayout = new QVBoxLayout(logGroup);

  logTextEdit_ = new QTextEdit();
  logTextEdit_->setFont(QFont("Consolas", 10));
  logTextEdit_->setReadOnly(true);
  logLayout->addWidget(logTextEdit_);

  // 로그 제어 버튼
  QHBoxLayout* logButtonLayout = new QHBoxLayout();

  clearLogButton_ = new QPushButton("로그 지우기");
  logButtonLayout->addWidget(clearLogButton_);

  saveLogButton_ = new QPushButton("로그 저장");
  logButtonLayout->addWidget(saveLogButton_);

  logButtonLayout->addStretch();
  logLayout->addLayout(logButtonLayout);

  layout->addWidget(logGroup);

  mainTabWidget_->addTab(logTab_, "로그");
}

void MainWindow::setupConnections() {
  // QNode 시그널-슬롯 연결
  connect(qnode_, &QNode::stateChanged, this, &MainWindow::onStateChanged);
  connect(qnode_, &QNode::cropsDetected, this, &MainWindow::onCropsDetected);
  connect(qnode_, &QNode::harvestOrderUpdated, this, &MainWindow::onHarvestOrderUpdated);
  connect(qnode_, &QNode::progressUpdated, this, &MainWindow::onProgressUpdated);
  connect(qnode_, &QNode::errorOccurred, this, &MainWindow::onErrorOccurred);
  connect(qnode_, &QNode::logMessage, this, &MainWindow::onLogMessage);
  connect(qnode_, &QNode::visionSystemReady, this, &MainWindow::onVisionSystemReady);
  connect(qnode_, &QNode::motorSystemReady, this, &MainWindow::onMotorSystemReady);
  connect(qnode_, &QNode::pathPlanningReady, this, &MainWindow::onPathPlanningReady);
  connect(qnode_, &QNode::robotStatusUpdated, this, &MainWindow::onRobotStatusUpdated);

  // 메인 제어 버튼
  connect(startButton_, &QPushButton::clicked, this, &MainWindow::onStartHarvest);
  connect(stopButton_, &QPushButton::clicked, this, &MainWindow::onStopHarvest);
  connect(emergencyButton_, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
  connect(resetButton_, &QPushButton::clicked, this, &MainWindow::onResetSystem);

  // 비전 시스템 버튼
  connect(triggerDetectionButton_, &QPushButton::clicked, this, &MainWindow::onTriggerDetection);
  connect(calibrateCameraButton_, &QPushButton::clicked, this, &MainWindow::onCalibrateSensors);

  // 모터 제어 버튼
  connect(moveButton_, &QPushButton::clicked, this, &MainWindow::onManualMove);
  connect(testCuttingButton_, &QPushButton::clicked, this, &MainWindow::onTestCutting);

  // 로그 버튼
  connect(clearLogButton_, &QPushButton::clicked, [this]() { logTextEdit_->clear(); });

  connect(saveLogButton_, &QPushButton::clicked, [this]() {
    QString fileName = QFileDialog::getSaveFileName(this, "로그 저장", QString("harvest_log_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")), "Text Files (*.txt)");
    if (!fileName.isEmpty()) {
      QFile file(fileName);
      if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << logTextEdit_->toPlainText();
        logMessage(QString("로그 저장 완료: %1").arg(fileName));
      }
    }
  });
}

// 슬롯 구현
void MainWindow::onStartHarvest() {
  if (!isHarvesting_) {
    qnode_->startHarvestSequence();
    isHarvesting_ = true;
    startButton_->setEnabled(false);
    stopButton_->setEnabled(true);
    logMessage("수확 시퀀스 시작 요청");
  }
}

void MainWindow::onStopHarvest() {
  if (isHarvesting_) {
    qnode_->stopHarvestSequence();
    isHarvesting_ = false;
    startButton_->setEnabled(true);
    stopButton_->setEnabled(false);
    logMessage("수확 시퀀스 중지 요청");
  }
}

void MainWindow::onEmergencyStop() {
  qnode_->emergencyStop();
  isHarvesting_ = false;
  startButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  showError("긴급 정지가 실행되었습니다!");
}

void MainWindow::onResetSystem() {
  qnode_->resetSystem();
  isHarvesting_ = false;
  startButton_->setEnabled(true);
  stopButton_->setEnabled(false);
  progressBar_->setValue(0);
  progressLabel_->setText("대기 중");
  cropsListWidget_->clear();
  harvestOrderWidget_->clear();
  currentTargetLabel_->setText("목표 없음");
  detectedCropsLabel_->setText("감지된 참외: 0개");
  logMessage("시스템 리셋 완료");
}

void MainWindow::onTriggerDetection() {
  qnode_->triggerVisionDetection();
  triggerDetectionButton_->setEnabled(false);
  QTimer::singleShot(3000, [this]() { triggerDetectionButton_->setEnabled(true); });
}

void MainWindow::onManualMove() {
  double x = xSpinBox_->value() / 1000.0;  // mm to m
  double y = ySpinBox_->value() / 1000.0;
  double z = zSpinBox_->value() / 1000.0;
  double rx = rxSpinBox_->value() * M_PI / 180.0;  // deg to rad
  double ry = rySpinBox_->value() * M_PI / 180.0;
  double rz = rzSpinBox_->value() * M_PI / 180.0;

  qnode_->moveToPosition(x, y, z, rx, ry, rz);
  moveButton_->setEnabled(false);
  QTimer::singleShot(2000, [this]() { moveButton_->setEnabled(true); });
}

void MainWindow::onTestCutting() {
  qnode_->activateCuttingTool();
  testCuttingButton_->setEnabled(false);
  QTimer::singleShot(5000, [this]() { testCuttingButton_->setEnabled(true); });
}

void MainWindow::onCalibrateSensors() {
  // 센서 캘리브레이션 로직
  logMessage("센서 캘리브레이션 시작");
  calibrateCameraButton_->setEnabled(false);
  QTimer::singleShot(10000, [this]() {
    calibrateCameraButton_->setEnabled(true);
    logMessage("센서 캘리브레이션 완료");
  });
}

void MainWindow::onStateChanged(QString state) {
  currentState_ = state;
  stateLabel_->setText(state);

  // 상태에 따른 색상 변경
  if (state == "READY") {
    stateLabel_->setStyleSheet("QLabel { background-color: #ccffcc; padding: 5px; border-radius: 3px; }");
  } else if (state == "HARVESTING" || state.contains("ING")) {
    stateLabel_->setStyleSheet("QLabel { background-color: #ccccff; padding: 5px; border-radius: 3px; }");
  } else if (state == "ERROR" || state == "EMERGENCY") {
    stateLabel_->setStyleSheet("QLabel { background-color: #ffcccc; padding: 5px; border-radius: 3px; }");
  } else {
    stateLabel_->setStyleSheet("QLabel { background-color: #ffffcc; padding: 5px; border-radius: 3px; }");
  }
}

void MainWindow::onCropsDetected(int count) {
  detectedCropsCount_ = count;
  detectedCropsLabel_->setText(QString("감지된 참외: %1개").arg(count));

  // 리스트 업데이트
  cropsListWidget_->clear();
  for (int i = 0; i < count; ++i) {
    cropsListWidget_->addItem(QString("참외 %1").arg(i + 1));
  }
}

void MainWindow::onHarvestOrderUpdated(QStringList order) {
  harvestOrderWidget_->clear();
  for (int i = 0; i < order.size(); ++i) {
    harvestOrderWidget_->addItem(QString("%1. %2").arg(i + 1).arg(order[i]));
  }

  if (!order.isEmpty()) {
    currentTargetLabel_->setText(QString("현재 목표: %1").arg(order[0]));
  }
}

void MainWindow::onProgressUpdated(int current, int total) {
  currentProgress_ = current;
  totalProgress_ = total;

  if (total > 0) {
    int percentage = (current * 100) / total;
    progressBar_->setValue(percentage);
    progressLabel_->setText(QString("진행: %1/%2 (%3%)").arg(current).arg(total).arg(percentage));

    if (current < total && current > 0) {
      // 다음 목표 업데이트
      QListWidgetItem* item = harvestOrderWidget_->item(current);
      if (item) {
        currentTargetLabel_->setText(QString("현재 목표: %1").arg(item->text()));
      }
    }
  }
}

void MainWindow::onErrorOccurred(QString error) {
  showError(error);
  logMessage(QString("오류: %1").arg(error));
}

void MainWindow::onLogMessage(QString message) { logMessage(message); }

void MainWindow::onVisionSystemReady() {
  visionStatusLabel_->setText("준비됨");
  visionStatusLabel_->setStyleSheet("QLabel { background-color: #ccffcc; padding: 5px; border-radius: 3px; }");
}

void MainWindow::onMotorSystemReady() {
  motorStatusLabel_->setText("준비됨");
  motorStatusLabel_->setStyleSheet("QLabel { background-color: #ccffcc; padding: 5px; border-radius: 3px; }");
}

void MainWindow::onPathPlanningReady() {
  planningStatusLabel_->setText("준비됨");
  planningStatusLabel_->setStyleSheet("QLabel { background-color: #ccffcc; padding: 5px; border-radius: 3px; }");
}

void MainWindow::onRobotStatusUpdated(QString status) {
  robotStatusLabel_->setText(status);
  if (status.contains("ERROR")) {
    robotStatusLabel_->setStyleSheet("QLabel { background-color: #ffcccc; padding: 5px; border-radius: 3px; }");
  } else {
    robotStatusLabel_->setStyleSheet("QLabel { background-color: #ccffcc; padding: 5px; border-radius: 3px; }");
  }
}

void MainWindow::updateSystemStatus() {
  // 주기적으로 시스템 상태 업데이트
  updateStatusIndicators();
}

void MainWindow::updateStatusIndicators() {
  // 각 시스템의 상태를 확인하고 표시기 업데이트
  QString systemOverall = "전체: ";
  if (qnode_->getCurrentState() == "READY") {
    systemOverall += "준비됨";
  } else if (isHarvesting_) {
    systemOverall += "수확 중";
  } else {
    systemOverall += qnode_->getCurrentState();
  }
}

void MainWindow::logMessage(const QString& message) {
  QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
  QString logEntry = QString("[%1] %2").arg(timestamp, message);
  logTextEdit_->append(logEntry);

  // 자동 스크롤
  logTextEdit_->verticalScrollBar()->setValue(logTextEdit_->verticalScrollBar()->maximum());
}

void MainWindow::showError(const QString& error) { QMessageBox::critical(this, "오류", error); }

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }