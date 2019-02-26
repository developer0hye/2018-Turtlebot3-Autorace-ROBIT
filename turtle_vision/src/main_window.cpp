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
#include "../include/turtle_vision/main_window.hpp"

namespace turtle_vision {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qRegisterMetaType<cv::Mat>("cv::Mat");
    uiInit();
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode.life_vision, SIGNAL(updateImg(cv::Mat)), this, SLOT(updateImg(cv::Mat)));
    QObject::connect(&qnode.life_vision, SIGNAL(updateLineImg(cv::Mat)), this, SLOT(updateLineImg(cv::Mat)));
    QObject::connect(&qnode.life_vision, SIGNAL(updateParkingLineImg(cv::Mat)), this, SLOT(updateParkingLineImg(cv::Mat)));
    QObject::connect(&qnode.life_vision, SIGNAL(updateTrafficLightRedImg(cv::Mat)), this, SLOT(updateTrafficLightRedImg(cv::Mat)));
    QObject::connect(&qnode.life_vision, SIGNAL(updateTrafficLightYellowImg(cv::Mat)), this, SLOT(updateTrafficLightYellowImg(cv::Mat)));
    QObject::connect(&qnode.life_vision, SIGNAL(updateTrafficLightGreenImg(cv::Mat)), this, SLOT(updateTrafficLightGreenImg(cv::Mat)));
    qnode.init();

    get_parameter();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::uiInit()
{
    ui.label_img_raw->setScaledContents(true);
    ui.label_img_line_roi->setScaledContents(true);
    ui.label_img_parking_line->setScaledContents(true);
    ui.label_img_traffic_light_red->setScaledContents(true);
    ui.label_img_traffic_light_yellow->setScaledContents(true);
    ui.label_img_traffic_light_green->setScaledContents(true);
}

void MainWindow::updateImg(cv::Mat img_raw)
{
    QImage Qimg_raw((const unsigned char*)(img_raw.data), img_raw.cols, img_raw.rows, QImage::Format_RGB888);
    ui.label_img_raw->setPixmap(QPixmap::fromImage(Qimg_raw.rgbSwapped()));
    //  QImage Qimg_raw((const unsigned char*)(img_raw.data), img_raw.cols, img_raw.rows, QImage::Format_Indexed8);
    //  ui.label_img_raw->setPixmap(QPixmap::fromImage(Qimg_raw));
}

void MainWindow::updateLineImg(cv::Mat imgLine)
{
    QImage QimgLine((const unsigned char*)(imgLine.data), imgLine.cols, imgLine.rows, QImage::Format_RGB888);
    ui.label_img_line_roi->setPixmap(QPixmap::fromImage(QimgLine.rgbSwapped()));
//      QImage QimgLine((const unsigned char*)(imgLine.data), imgLine.cols, imgLine.rows, imgLine.step, QImage::Format_Indexed8);
//     ui.label_img_line_roi->setPixmap(QPixmap::fromImage(QimgLine));
}

void MainWindow::updateTrafficLightRedImg(cv::Mat imgTrafficLightRed)
{
  QImage QimgTrafficLightRed((const unsigned char*)(imgTrafficLightRed.data), imgTrafficLightRed.cols, imgTrafficLightRed.rows, imgTrafficLightRed.step, QImage::Format_Indexed8);
  ui.label_img_traffic_light_red->setPixmap(QPixmap::fromImage(QimgTrafficLightRed));
}

void MainWindow::updateTrafficLightYellowImg(cv::Mat imgTrafficLightYellow)
{
  QImage QimgTrafficLightYellow((const unsigned char*)(imgTrafficLightYellow.data), imgTrafficLightYellow.cols, imgTrafficLightYellow.rows, imgTrafficLightYellow.step, QImage::Format_Indexed8);
  ui.label_img_traffic_light_yellow->setPixmap(QPixmap::fromImage(QimgTrafficLightYellow));
}

void MainWindow::updateTrafficLightGreenImg(cv::Mat imgTrafficLightGreen)
{
  QImage QimgTrafficLightGreen((const unsigned char*)(imgTrafficLightGreen.data), imgTrafficLightGreen.cols, imgTrafficLightGreen.rows, imgTrafficLightGreen.step, QImage::Format_Indexed8);
  ui.label_img_traffic_light_green->setPixmap(QPixmap::fromImage(QimgTrafficLightGreen));
}

void MainWindow::updateParkingLineImg(cv::Mat imgLineParking)
{
    QImage QimgLine((const unsigned char*)(imgLineParking.data), imgLineParking.cols, imgLineParking.rows, QImage::Format_RGB888);
    ui.label_img_parking_line->setPixmap(QPixmap::fromImage(QimgLine.rgbSwapped()));
}

void MainWindow::updateTunnelImg(cv::Mat imgTunnel)
{

}
void MainWindow::get_parameter()
{
    ifstream fin_default_color("/home/turtle1/catkin_ws/src/turtle_vision/turtle_vision/color_parameter/color_parameter.txt");

    if(fin_default_color.is_open())
    {
        std::cout << "open" << std::endl;
        fin_default_color >> qnode.life_vision.pogba_hue_min
                >> qnode.life_vision.pogba_hue_max
                >> qnode.life_vision.pogba_sat_min
                >> qnode.life_vision.pogba_sat_max
                >> qnode.life_vision.pogba_val_min
                >> qnode.life_vision.pogba_val_max

                >> qnode.life_vision.traffic_light_red_hue_min
                >> qnode.life_vision.traffic_light_red_hue_max
                >> qnode.life_vision.traffic_light_red_sat_min
                >> qnode.life_vision.traffic_light_red_sat_max
                >> qnode.life_vision.traffic_light_red_val_min
                >> qnode.life_vision.traffic_light_red_val_max

                >> qnode.life_vision.traffic_light_yellow_hue_min
                >> qnode.life_vision.traffic_light_yellow_hue_max
                >> qnode.life_vision.traffic_light_yellow_sat_min
                >> qnode.life_vision.traffic_light_yellow_sat_max
                >> qnode.life_vision.traffic_light_yellow_val_min
                >> qnode.life_vision.traffic_light_yellow_val_max

                >>qnode.life_vision.traffic_light_red_Y
                >>qnode.life_vision.traffic_light_yellow_Y

                >> qnode.life_vision.traffic_light_green_hue_min
                >> qnode.life_vision.traffic_light_green_hue_max
                >> qnode.life_vision.traffic_light_green_sat_min
                >> qnode.life_vision.traffic_light_green_sat_max
                >> qnode.life_vision.traffic_light_green_val_min
                >> qnode.life_vision.traffic_light_green_val_max

                >>qnode.life_vision.traffic_light_green_Y;

        fin_default_color.close();


        on_horizontalSlider_red_Y_valueChanged(qnode.life_vision.traffic_light_red_Y);
        on_horizontalSlider_yellow_Y_valueChanged(qnode.life_vision.traffic_light_yellow_Y);
        on_horizontalSlider_green_Y_valueChanged(qnode.life_vision.traffic_light_green_Y);
        ui.horizontalSlider_red_Y->setValue(qnode.life_vision.traffic_light_red_Y);
        ui.horizontalSlider_yellow_Y->setValue(qnode.life_vision.traffic_light_yellow_Y);
        ui.horizontalSlider_green_Y->setValue(qnode.life_vision.traffic_light_green_Y);
    }

    else
    {
        std::cout << " default_color.txt is close " << std::endl;
    }
}
void MainWindow::on_radioButton_pogba_clicked()
{
    std::cout << "pogba open" << std::endl;

    on_horizontalSlider_hue_min_valueChanged(qnode.life_vision.pogba_hue_min);
    on_horizontalSlider_hue_max_valueChanged(qnode.life_vision.pogba_hue_max);
    on_horizontalSlider_sat_min_valueChanged(qnode.life_vision.pogba_sat_min);
    on_horizontalSlider_sat_max_valueChanged(qnode.life_vision.pogba_sat_max);
    on_horizontalSlider_val_min_valueChanged(qnode.life_vision.pogba_val_min);
    on_horizontalSlider_val_max_valueChanged(qnode.life_vision.pogba_val_max);

    ui.horizontalSlider_hue_min->setValue(qnode.life_vision.pogba_hue_min);
    ui.horizontalSlider_hue_max->setValue(qnode.life_vision.pogba_hue_max);
    ui.horizontalSlider_sat_min->setValue(qnode.life_vision.pogba_sat_min);
    ui.horizontalSlider_sat_max->setValue(qnode.life_vision.pogba_sat_max);
    ui.horizontalSlider_val_min->setValue(qnode.life_vision.pogba_val_min);
    ui.horizontalSlider_val_max->setValue(qnode.life_vision.pogba_val_max);
}

void MainWindow::on_radioButton_traffic_light_red_clicked()
{
    std::cout << "traffic light open" << std::endl;

    on_horizontalSlider_hue_min_valueChanged(qnode.life_vision.traffic_light_red_hue_min);
    on_horizontalSlider_hue_max_valueChanged(qnode.life_vision.traffic_light_red_hue_max);
    on_horizontalSlider_sat_min_valueChanged(qnode.life_vision.traffic_light_red_sat_min);
    on_horizontalSlider_sat_max_valueChanged(qnode.life_vision.traffic_light_red_sat_max);
    on_horizontalSlider_val_min_valueChanged(qnode.life_vision.traffic_light_red_val_min);
    on_horizontalSlider_val_max_valueChanged(qnode.life_vision.traffic_light_red_val_max);

    ui.horizontalSlider_hue_min->setValue(qnode.life_vision.traffic_light_red_hue_min);
    ui.horizontalSlider_hue_max->setValue(qnode.life_vision.traffic_light_red_hue_max);
    ui.horizontalSlider_sat_min->setValue(qnode.life_vision.traffic_light_red_sat_min);
    ui.horizontalSlider_sat_max->setValue(qnode.life_vision.traffic_light_red_sat_max);
    ui.horizontalSlider_val_min->setValue(qnode.life_vision.traffic_light_red_val_min);
    ui.horizontalSlider_val_max->setValue(qnode.life_vision.traffic_light_red_val_max);
}

void MainWindow::on_radioButton_traffic_light_yellow_clicked()
{
    std::cout << "traffic light open" << std::endl;

    on_horizontalSlider_hue_min_valueChanged(qnode.life_vision.traffic_light_yellow_hue_min);
    on_horizontalSlider_hue_max_valueChanged(qnode.life_vision.traffic_light_yellow_hue_max);
    on_horizontalSlider_sat_min_valueChanged(qnode.life_vision.traffic_light_yellow_sat_min);
    on_horizontalSlider_sat_max_valueChanged(qnode.life_vision.traffic_light_yellow_sat_max);
    on_horizontalSlider_val_min_valueChanged(qnode.life_vision.traffic_light_yellow_val_min);
    on_horizontalSlider_val_max_valueChanged(qnode.life_vision.traffic_light_yellow_val_max);

    ui.horizontalSlider_hue_min->setValue(qnode.life_vision.traffic_light_yellow_hue_min);
    ui.horizontalSlider_hue_max->setValue(qnode.life_vision.traffic_light_yellow_hue_max);
    ui.horizontalSlider_sat_min->setValue(qnode.life_vision.traffic_light_yellow_sat_min);
    ui.horizontalSlider_sat_max->setValue(qnode.life_vision.traffic_light_yellow_sat_max);
    ui.horizontalSlider_val_min->setValue(qnode.life_vision.traffic_light_yellow_val_min);
    ui.horizontalSlider_val_max->setValue(qnode.life_vision.traffic_light_yellow_val_max);
}

void MainWindow::on_radioButton_traffic_light_green_clicked()
{
    std::cout << "traffic light open" << std::endl;

    on_horizontalSlider_hue_min_valueChanged(qnode.life_vision.traffic_light_green_hue_min);
    on_horizontalSlider_hue_max_valueChanged(qnode.life_vision.traffic_light_green_hue_max);
    on_horizontalSlider_sat_min_valueChanged(qnode.life_vision.traffic_light_green_sat_min);
    on_horizontalSlider_sat_max_valueChanged(qnode.life_vision.traffic_light_green_sat_max);
    on_horizontalSlider_val_min_valueChanged(qnode.life_vision.traffic_light_green_val_min);
    on_horizontalSlider_val_max_valueChanged(qnode.life_vision.traffic_light_green_val_max);

    ui.horizontalSlider_hue_min->setValue(qnode.life_vision.traffic_light_green_hue_min);
    ui.horizontalSlider_hue_max->setValue(qnode.life_vision.traffic_light_green_hue_max);
    ui.horizontalSlider_sat_min->setValue(qnode.life_vision.traffic_light_green_sat_min);
    ui.horizontalSlider_sat_max->setValue(qnode.life_vision.traffic_light_green_sat_max);
    ui.horizontalSlider_val_min->setValue(qnode.life_vision.traffic_light_green_val_min);
    ui.horizontalSlider_val_max->setValue(qnode.life_vision.traffic_light_green_val_max);
}

void MainWindow::on_pushButton_parameter_save_clicked()
{
    ofstream fout_default_color("/home/turtle1/catkin_ws/src/turtle_vision/turtle_vision/color_parameter/color_parameter.txt");

    if(fout_default_color.is_open())
    {
        std::cout << "SAVE !!!" << std::endl;

        fout_default_color << qnode.life_vision.pogba_hue_min << endl
                           << qnode.life_vision.pogba_hue_max<< endl
                           << qnode.life_vision.pogba_sat_min<< endl
                           << qnode.life_vision.pogba_sat_max<< endl
                           << qnode.life_vision.pogba_val_min<< endl
                           << qnode.life_vision.pogba_val_max<< endl

                           << qnode.life_vision.traffic_light_red_hue_min<< endl
                           << qnode.life_vision.traffic_light_red_hue_max<< endl
                           << qnode.life_vision.traffic_light_red_sat_min<< endl
                           << qnode.life_vision.traffic_light_red_sat_max<< endl
                           << qnode.life_vision.traffic_light_red_val_min<< endl
                           << qnode.life_vision.traffic_light_red_val_max<< endl

                           << qnode.life_vision.traffic_light_yellow_hue_min<< endl
                           << qnode.life_vision.traffic_light_yellow_hue_max<< endl
                           << qnode.life_vision.traffic_light_yellow_sat_min<< endl
                           << qnode.life_vision.traffic_light_yellow_sat_max<< endl
                           << qnode.life_vision.traffic_light_yellow_val_min<< endl
                           << qnode.life_vision.traffic_light_yellow_val_max<< endl

                           <<qnode.life_vision.traffic_light_red_Y<< endl
                           <<qnode.life_vision.traffic_light_yellow_Y<< endl

                          << qnode.life_vision.traffic_light_green_hue_min<< endl
                          << qnode.life_vision.traffic_light_green_hue_max<< endl
                          << qnode.life_vision.traffic_light_green_sat_min<< endl
                          << qnode.life_vision.traffic_light_green_sat_max<< endl
                          << qnode.life_vision.traffic_light_green_val_min<< endl
                          << qnode.life_vision.traffic_light_green_val_max<< endl

                          <<qnode.life_vision.traffic_light_green_Y<< endl;
    }
}
void MainWindow::on_horizontalSlider_hue_min_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_hue_min = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_hue_min = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_hue_min = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_hue_min = value;
    ui.lineEdit_hue_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_hue_max_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_hue_max = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_hue_max = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_hue_max = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_hue_max = value;

    ui.lineEdit_hue_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_sat_min_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_sat_min = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_sat_min = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_sat_min = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_sat_min = value;

    ui.lineEdit_sat_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_sat_max_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_sat_max = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_sat_max = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_sat_max = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_sat_max = value;

    ui.lineEdit_sat_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_val_min_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_val_min = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_val_min = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_val_min = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_val_min = value;

    ui.lineEdit_val_min->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_val_max_valueChanged(int value)
{
    if(ui.radioButton_pogba->isChecked())
        qnode.life_vision.pogba_val_max = value;
    else if(ui.radioButton_traffic_light_red->isChecked())
        qnode.life_vision.traffic_light_red_val_max = value;
    else if(ui.radioButton_traffic_light_yellow->isChecked())
        qnode.life_vision.traffic_light_yellow_val_max = value;
    else if(ui.radioButton_traffic_light_green->isChecked())
        qnode.life_vision.traffic_light_green_val_max = value;


    ui.lineEdit_val_max->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_red_Y_valueChanged(int value)
{
    qnode.life_vision.traffic_light_red_Y = value;
    ui.lineEdit_red_Y->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_yellow_Y_valueChanged(int value)
{
    qnode.life_vision.traffic_light_yellow_Y = value;
    ui.lineEdit_yellow_Y->setText(QString::number(value));
}

void MainWindow::on_horizontalSlider_green_Y_valueChanged(int value)
{
    qnode.life_vision.traffic_light_green_Y = value;
    ui.lineEdit_green_Y->setText(QString::number(value));
}


}  // namespace turtle_vision

