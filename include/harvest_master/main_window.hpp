/**
 * @file /include/harvest_master/main_window.hpp
 *
 * @brief Qt based gui for harvest_master.
 *
 * @date January 2025
 **/

 #ifndef harvest_master_MAIN_WINDOW_H
 #define harvest_master_MAIN_WINDOW_H
 
 /*****************************************************************************
 ** Includes
 *****************************************************************************/
 
 #include <QMainWindow>
 #include <QTimer>
 #include <QTime>
 #include <QTextCursor>
 #include <QTextDocument>
 #include <QPushButton>
 #include <QLabel>
 #include <QProgressBar>
 #include <QTextEdit>
 #include "QIcon"
 #include "qnode.hpp"
 #include "ui_mainwindow.h"
 
 /*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
 /**
  * @brief Qt central, all operations relating to the view part here.
  */
 class MainWindow : public QMainWindow
 {
   Q_OBJECT
 
 public:
   MainWindow(QWidget* parent = nullptr);
   ~MainWindow();
   QNode* qnode;
 
 private slots:
   // Vision System Slots
   void onStartVision();
   void onCapture();
   void onCalibrate();
   
   // Manipulation System Slots
   void onStartHarvest();
   void onMoveToTarget();
   void onGrip();
   void onMoveHome();
   void onMoveToDrop();
   
   // Emergency Control Slots
   void onEmergencyStop();
   void onResetArm();
   void onResetVision();
   
   // Update Slot
   void updateUI();
   
   // ROS2 Signal Slots
   void onImageReceived(const QPixmap& pixmap);
   void onMelonDetected(int count, int ripe_count);
   void onSystemStatusChanged(const QString& status);
   void onTargetSelected(double x, double y);
 
 private:
   Ui::MainWindowDesign* ui;
   QTimer* updateTimer;
   
   // System state variables
   int harvestCount;
   int successCount;
   bool isVisionActive;
   bool isArmReady;
   
   // Helper functions
   void setupConnections();
   void setupTimers();
   void initializeUI();
   void appendSystemMessage(const QString& type, const QString& message, const QString& color);
   void closeEvent(QCloseEvent* event);
 };
 
 #endif  // harvest_master_MAIN_WINDOW_H