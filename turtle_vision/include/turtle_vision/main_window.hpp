/**
 * @file /include/turtle_vision/main_window.hpp
 *
 * @brief Qt based gui for turtle_vision.
 *
 * @date November 2010
 **/
#ifndef turtle_vision_MAIN_WINDOW_H
#define turtle_vision_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include <iostream>
#include <fstream>
#include "ui_main_window.h"
#include "qnode.hpp"


namespace turtle_vision {

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
    void updateImg(cv::Mat imgRaw);
    void updateLineImg(cv::Mat imgLine);
    void updateTrafficLightRedImg(cv::Mat imgTrafficLightRed);
    void updateTrafficLightYellowImg(cv::Mat imgTrafficLightYellow);
    void updateTrafficLightGreenImg(cv::Mat imgTrafficLightGreen);
    void updateParkingLineImg(cv::Mat imgLineParking);
    void updateTunnelImg(cv::Mat imgTunnel);

    void on_radioButton_pogba_clicked();
    void on_radioButton_traffic_light_red_clicked();
    void on_radioButton_traffic_light_yellow_clicked();
    void on_radioButton_traffic_light_green_clicked();
    void on_pushButton_parameter_save_clicked();

    void on_horizontalSlider_hue_min_valueChanged(int value);
    void on_horizontalSlider_hue_max_valueChanged(int value);
    void on_horizontalSlider_sat_min_valueChanged(int value);
    void on_horizontalSlider_sat_max_valueChanged(int value);
    void on_horizontalSlider_val_min_valueChanged(int value);
    void on_horizontalSlider_val_max_valueChanged(int value);
    void on_horizontalSlider_red_Y_valueChanged(int value);
    void on_horizontalSlider_yellow_Y_valueChanged(int value);
    void on_horizontalSlider_green_Y_valueChanged(int value);


private:
    Ui::MainWindowDesign ui;
    QNode qnode;

    void uiInit();
    void get_parameter();
};

}  // namespace turtle_vision

#endif // turtle_vision_MAIN_WINDOW_H
