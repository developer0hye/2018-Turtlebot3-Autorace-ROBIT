/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/turtle_master/main_window.hpp"

namespace turtle_master {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(turtlebot3_state_update()), this, SLOT(turtlebot3_state_update()));
  qnode.init();
}
MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::turtlebot3_state_update()
{
  ui.lineEdit_left_degree->setText(QString::number(qnode.life_driving.left_line_degree));
  ui.lineEdit_right_degree->setText(QString::number(qnode.life_driving.right_line_degree));

  ui.lineEdit_right_start_x->setText(QString::number(qnode.life_driving.right_line_start.x));
  ui.lineEdit_right_start_y->setText(QString::number(qnode.life_driving.right_line_start.y));

  ui.lineEdit_left_start_x->setText(QString::number(qnode.life_driving.left_line_start.x));
  ui.lineEdit_left_start_y->setText(QString::number(qnode.life_driving.left_line_start.y));

  ui.lineEdit_right_end_x->setText(QString::number(qnode.life_driving.right_line_end.x));
  ui.lineEdit_right_end_y->setText(QString::number(qnode.life_driving.right_line_end.y));

  ui.lineEdit_left_end_x->setText(QString::number(qnode.life_driving.left_line_end.x));
  ui.lineEdit_left_end_y->setText(QString::number(qnode.life_driving.left_line_end.y));


  ui.lineEdit_imu_data->setText(QString::number(qnode.my_imu));
  ui.lineEdit_psd_data->setText(QString::number(qnode.life_driving.psd_val));
  ui.lineEdit_cds_data->setText(QString::number(qnode.life_driving.cds_val));
  ui.lineEdit_lidar_data->setText(QString::number(qnode.life_driving.lidarDistance));
  ui.lineEdit_robot_state->setText(qnode.missionState);
}

void MainWindow::on_pushButton_start_clicked()
{
    /*if(qnode.bStartAtuoDriving)
    {
        qnode.bStartAtuoDriving = false;
    }
    else
    {
        qnode.bStartAtuoDriving = true;
    }*/


    if(qnode.life_driving.drive_state != LifeDriving::TURTLE_STOP)
    {
        ui.pushButton_start->setText("STOP");
        qnode.life_driving.past_drvie_state = qnode.life_driving.drive_state;
        qnode.life_driving.drive_state = LifeDriving::TURTLE_STOP;
    }
    else //cur drive state == STOP
    {
        ui.pushButton_start->setText("GO");
        if(qnode.life_driving.past_drvie_state == LifeDriving::TURTLE_STOP)
        {
            qnode.life_driving.drive_state = LifeDriving::TURTLE_TRAFFIC_LIGHT_CHECK;
        }
        else
        {
            qnode.life_driving.drive_state = qnode.life_driving.past_drvie_state;
        }
    }
}
}  // namespace turtle_master

