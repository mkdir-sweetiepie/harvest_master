// include/harvest_master/main_window.hpp
#ifndef HARVEST_MASTER_MAIN_WINDOW_HPP_
#define HARVEST_MASTER_MAIN_WINDOW_HPP_

#include <QCloseEvent>
#include <QtCore/QDateTime>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>

#include "qnode.hpp"
#include "ui_mainwindow.h"

namespace Ui {
class MainWindowDesign;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 protected:
  void closeEvent(QCloseEvent* event) override;

 private Q_SLOTS:
  // 메인 제어 슬롯
  void onStartHarvest();
  void onStopHarvest();
  void onEmergencyStop();
  void onResetSystem();

  // 개별 모듈 제어 슬롯
  void onTriggerDetection();
  void onManualMove();
  void onTestCutting();
  void onCalibrateSensors();

  // QNode 시그널 처리 슬롯
  void onStateChanged(QString state);
  void onCropsDetected(int count);
  void onHarvestOrderUpdated(QStringList order);
  void onProgressUpdated(int current, int total);
  void onErrorOccurred(QString error);
  void onLogMessage(QString message);

  // 시스템 상태 업데이트 슬롯
  void onVisionSystemReady();
  void onMotorSystemReady();
  void onPathPlanningReady();
  void onRobotStatusUpdated(QString status);

  // 타이머 슬롯
  void updateSystemStatus();

 private:
  void setupUI();
  void setupControlTab();
  void setupVisionTab();
  void setupMotorTab();
  void setupPlanningTab();
  void setupLogTab();
  void setupConnections();
  void updateStatusIndicators();
  void logMessage(const QString& message);
  void showError(const QString& error);

  Ui::MainWindowDesign* ui;
  QNode* qnode_;

  // UI 컴포넌트들
  QTabWidget* mainTabWidget_;

  // 메인 제어 탭
  QWidget* controlTab_;
  QPushButton* startButton_;
  QPushButton* stopButton_;
  QPushButton* emergencyButton_;
  QPushButton* resetButton_;
  QLabel* stateLabel_;
  QProgressBar* progressBar_;
  QLabel* progressLabel_;

  // 시스템 상태 표시
  QGroupBox* statusGroup_;
  QLabel* visionStatusLabel_;
  QLabel* motorStatusLabel_;
  QLabel* planningStatusLabel_;
  QLabel* robotStatusLabel_;

  // 비전 시스템 탭
  QWidget* visionTab_;
  QPushButton* triggerDetectionButton_;
  QLabel* detectedCropsLabel_;
  QListWidget* cropsListWidget_;
  QPushButton* calibrateCameraButton_;

  // 모터 제어 탭
  QWidget* motorTab_;
  QGroupBox* manualControlGroup_;
  QDoubleSpinBox* xSpinBox_;
  QDoubleSpinBox* ySpinBox_;
  QDoubleSpinBox* zSpinBox_;
  QDoubleSpinBox* rxSpinBox_;
  QDoubleSpinBox* rySpinBox_;
  QDoubleSpinBox* rzSpinBox_;
  QPushButton* moveButton_;
  QPushButton* testCuttingButton_;

  // 경로 계획 탭
  QWidget* planningTab_;
  QListWidget* harvestOrderWidget_;
  QLabel* currentTargetLabel_;
  QPushButton* recalculatePathButton_;

  // 로그 및 디버그 탭
  QWidget* logTab_;
  QTextEdit* logTextEdit_;
  QPushButton* clearLogButton_;
  QPushButton* saveLogButton_;

  // 상태 업데이트 타이머
  QTimer* statusUpdateTimer_;

  // 상태 변수
  bool isHarvesting_;
  QString currentState_;
  int detectedCropsCount_;
  int currentProgress_;
  int totalProgress_;
};

#endif  // HARVEST_MASTER_MAIN_WINDOW_HPP_