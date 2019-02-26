#ifndef CANNYEDGE_YH_HPP
#define CANNYEDGE_YH_HPP

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

namespace yh
{
    class CannyEdge
    {
        const static int CERTAIN_EDGE = 255;
        const static int PROBABLE_EDGE = 127;

    public:
        CannyEdge(cv::Mat& src,
                  double weight_dX = 1.0,
                  double weight_dY = 1.0,
                  bool bUseColorEdge = false);

        void detection(int th_low = 0, int th_high = 0);
        void makeLine();
        void makeVerticalLine();


        cv::Mat edge;

    private:

        void computeMagnitude();
        void computeDirection();

        void nonMaximumSupression();

        void adaptiveThreshold(int& th_low, int& th_high);
        void hysteresisThresholding(int th_low, int th_high);

        double w_dX;
        double w_dY;

        cv::Mat filtered;
        cv::Mat dX;
        cv::Mat dY;
        cv::Mat mag;
        cv::Mat dir;
        cv::Mat nms;

        std::vector<double> edgeCandidates;
    };
}
#endif // CANNYEDGE_YH_HPP
