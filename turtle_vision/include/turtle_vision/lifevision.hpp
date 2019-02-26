#ifndef LIFEVISION_H
#define LIFEVISION_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <QObject>
#include <QTimer>
#include "../../devel/include/turtle_vision/vision_msg.h"

#include "robit_ransac.hpp"

#include "darknetdetector.hpp"

#include "siftusingknn.hpp"
#include "cannyedge_yh.hpp"
#include "trafficlightrecognition.hpp"

class LifeVision : public QObject{

    Q_OBJECT
public:
    explicit LifeVision(QObject *parent = 0);
    ~LifeVision();
    void process(cv::Mat& src);

    turtle_vision::vision_msg vision_msg;
    cv::Mat img_line_edge;

    int pogba_hue_min;
    int pogba_hue_max;
    int pogba_sat_min;
    int pogba_sat_max;
    int pogba_val_min;
    int pogba_val_max;

    int traffic_light_red_hue_min;
    int traffic_light_red_hue_max;
    int traffic_light_red_sat_min;
    int traffic_light_red_sat_max;
    int traffic_light_red_val_min;
    int traffic_light_red_val_max;

    int traffic_light_yellow_hue_min;
    int traffic_light_yellow_hue_max;
    int traffic_light_yellow_sat_min;
    int traffic_light_yellow_sat_max;
    int traffic_light_yellow_val_min;
    int traffic_light_yellow_val_max;


    int traffic_light_green_hue_min;
    int traffic_light_green_hue_max;
    int traffic_light_green_sat_min;
    int traffic_light_green_sat_max;
    int traffic_light_green_val_min;
    int traffic_light_green_val_max;

    int traffic_light_red_Y;
    int traffic_light_yellow_Y;
    int traffic_light_green_Y;

    int psd_val;
    int victory_data;
    bool parking_mode;

private:
    cv::Point2f src_vertices[4];
    cv::Point2f dst_vertices[4];

    cv::Point2f src_vertices_parking[4];
    cv::Point2f dst_vertices_parking[4];

    int  parking_mark_cnt;
    int  parking_line_cnt;
    bool parking_line_start;
    bool parking_line_flag;

    int  across_mark_cnt;
    bool across_flag;
    SiftUsingKNN siftParking;
    SiftUsingKNN siftAcrossLeft;
    SiftUsingKNN siftAcrossRight;

    cv::Mat srcDrawing;

    enum {POGBA = 1, GREEN_LIGHT, PARKING, ACROSS_LEFT, ACROSS_RIGHT};

    enum {NOT_DETECTED = -1};

    void _lineProcess           (cv::Mat& src);
    void __lineJudge            (cv::Mat& src, cv::Mat &edge, cv::Mat& leftLine, cv::Mat& rightLine, bool parking_mode);
    void __lineExtraction       (cv::Mat& src, cv::Mat& leftLine, cv::Mat& rightLine);

    void _trafficLightProcess   (cv::Mat& src);
    void _pogbaProcess          (cv::Mat& src);
    void _parkingProcess        (cv::Mat& src);
    void _acrossProcess         (cv::Mat& src);
    void _stopSignProcess       (cv::Mat& src);

    void _drawHelpInform(cv::Mat& src);

Q_SIGNALS:
    void updateImg                  (cv::Mat imgRaw);
    void updateLineImg              (cv::Mat imgLine);
    void updateTrafficLightRedImg   (cv::Mat imgTrafficLightRed);
    void updateTrafficLightYellowImg(cv::Mat imgTrafficLightYellow);
    void updateTrafficLightGreenImg (cv::Mat imgTrafficLightGreen);
    void updateParkingLineImg       (cv::Mat imgParkingLine);
    void updateParkingImg           (cv::Mat imgParking);
};
#endif // LIFEVISION_H
