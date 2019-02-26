#ifndef SIFTUSINGKNN_H
#define SIFTUSINGKNN_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv::xfeatures2d;

class SiftUsingKNN
{

public:
    SiftUsingKNN()
        :keypointsNum(0)
        {}
    SiftUsingKNN(cv::Mat object, int minHessian = 1000)
        :
          img_object(object),
          objPoint(0,0),
          keypointsNum(0)
    {
        color = cv::Scalar(0, 255, 0);

        sift = SIFT::create(minHessian);
        sift->detectAndCompute( img_object, cv::noArray(), keypoints_object, descriptors_object );

        matchingPointThreshold = std::min( (int)(keypoints_object.size()/5) , 4);
    }
    ~SiftUsingKNN(){;}

    void setRoi(cv::Rect roi){this->roi = roi;}
    void setColor(cv::Scalar color){this->color = color;}
    bool process(cv::Mat& scene, cv::Rect roi , bool viewMatches);
    void localizeInImage(const std::vector<cv::DMatch>& good_matches,
                         const std::vector<cv::KeyPoint>& keypoints_object,
                         const std::vector<cv::KeyPoint>& keypoints_scene, const cv::Mat& img_object,
                         const cv::Mat& img_matches, cv::Mat& img_scene, cv::Rect roi);

    cv::Point objPoint;
    int       keypointsNum;

private:

    cv::Mat img_object;

    cv::Rect roi;

    cv::Ptr<SIFT>           sift;
    std::vector<cv::KeyPoint>    keypoints_object;
    cv::Mat                 descriptors_object;
    cv::Scalar              color;

    int matchingPointThreshold;
};

#endif // SIFTUSINGKNN_H
