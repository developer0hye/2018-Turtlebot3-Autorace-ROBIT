#ifndef TRAFFICLIGHTRECOGNITION_HPP
#define TRAFFICLIGHTRECOGNITION_HPP

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

class TrafficLightRecognition
{

public:
    TrafficLightRecognition()
    {
        this->trafficLightMaxSize           = 350;
        this->trafficLightMinSize           = 50;
        this->labledRatio                   = 0.7;
        this->widthHeightRatioWeight        = 0.5;
    }
    cv::Mat trafficLight[3];
    void setRedLight(cv::Scalar colorL, cv::Scalar colorH){redLightL = colorL; redLightH = colorH;}
    void setYellowLight(cv::Scalar colorL, cv::Scalar colorH){yellowLightL = colorL; yellowLightH = colorH;}
    void setGreenLight(cv::Scalar colorL, cv::Scalar colorH){greenLightL = colorL; greenLightH = colorH;}


    void setThreshold(int trafficLightMaxSize,
                      int trafficLightMinSize,
                      double labledRatio,
                      double widthHeightRatioWeight)
    {
        this->trafficLightMaxSize  = trafficLightMaxSize;
        this->trafficLightMinSize  = trafficLightMinSize;
        this->labledRatio       = labledRatio;
        this->widthHeightRatioWeight       = widthHeightRatioWeight;
    }

    void recognition(cv::Mat src, cv::Rect &roiTrafficLight,  cv::Point& lightCenter, int &light);
    void recognitionBasedYolo(cv::Mat src,int &light);

private:


    int     trafficLightMinSize;
    int     trafficLightMaxSize;
    double  labledRatio;
    double  widthHeightRatioWeight;


    cv::Scalar redLightL, redLightH;
    cv::Scalar yellowLightL, yellowLightH;
    cv::Scalar greenLightL, greenLightH;
};
#endif // TRAFFICLIGHTRECOGNITION_HPP
