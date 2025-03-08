#include <QApplication>
#include <iostream>

#include "../include/harvest_master/main_window.hpp"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  // 애플리케이션 정보 설정
  app.setApplicationName("BARAM 농작물 수확 로봇 관제 시스템");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("Robotics Club BARAM");

  MainWindow window;
  window.show();

  return app.exec();
}