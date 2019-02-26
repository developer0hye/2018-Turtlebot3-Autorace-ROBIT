/**
 * @file /include/turtle_master/main_window.hpp
 *
 * @brief Qt based gui for turtle_master.
 *
 * @date November 2010
 **/
#ifndef turtle_master_MAIN_WINDOW_H
#define turtle_master_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include <QString>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "life_driving.hpp"

namespace turtle_master {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
  void turtlebot3_state_update();
  void on_pushButton_start_clicked();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace turtle_master

#endif // turtle_master_MAIN_WINDOW_H
