#include "../include/turtle_vision/cannyedge_yh.hpp"

using namespace yh;
using namespace std;

CannyEdge::CannyEdge(cv::Mat& src,
                     double weight_dX,
                     double weight_dY,
                     bool bUseColorEdge)
    :
      w_dX(weight_dX),
      w_dY(weight_dY)
{
    const int kernel_size = 3;
    const int scale = 1;
    const int delta = 0;
    const int ddepth = CV_32F;

    //const int lpf_kernel_length = 21;
    //bilateralFilter(src, filtered, lpf_kernel_length, lpf_kernel_length*2, lpf_kernel_length/2);

    cv::medianBlur(src, filtered, 5);
    cv::GaussianBlur(src, filtered, cv::Size(13, 13), 2.0);

    if(bUseColorEdge)
    {

    }
    else
    {
        if(filtered.channels() == 3)
        {
            cv::Mat srcLuv, srcLuvCH[3];
            cv::cvtColor(filtered, srcLuv, CV_BGR2Luv);

            cv::split(srcLuv,srcLuvCH);
            filtered = srcLuvCH[0].clone();
        }

        // Gradient X and Y
        cv::Sobel( filtered, dX, ddepth, 1, 0, kernel_size, scale, delta);
        cv::Sobel( filtered, dY, ddepth, 0, 1, kernel_size, scale, delta);

        computeMagnitude();
        computeDirection();
        nonMaximumSupression();
    }
}

void CannyEdge::computeMagnitude()
{
    mag = cv::Mat::zeros(filtered.size(), CV_32F);

    for(int i = 1; i < mag.rows - 1; i++)
        for(int j = 1; j < mag.cols - 1; j++)
        {
            // mag.at<float>(i, j) = fabs(dX.at<float>(i, j))* w_dX + fabs(dY.at<float>(i, j))* w_dY;
            mag.at<float>(i, j) = sqrt(pow(dX.at<float>(i, j)* w_dX,2.0) + pow(dY.at<float>(i, j)* w_dY,2.0));
        }
}

void CannyEdge::computeDirection()
{
    cv::phase(dX, dY, dir, true);
}

void CannyEdge::nonMaximumSupression()
{
    nms = cv::Mat::zeros(mag.rows, mag.cols, CV_32F);

    for(int i = 1; i < mag.rows - 1; i++)
        for(int j = 1; j < mag.cols - 1; j++)
        {
            double  ang     = dir.at<float>(i, j);
            float   fMag    = mag.at<float>(i, j);
            bool    bMaxima = true;

            if((ang <= 22.5 || ang > 337.5) || ( ang > 157.5 && ang <= 202.5)) //horizontal
            {
                if(fMag < mag.at<float>(i, j -1) || fMag < mag.at<float>(i, j + 1))
                {
                    bMaxima = false;
                }
            }
            else if((ang > 22.5 && ang <= 67.5) || ( ang > 202.5 && ang <= 247.5)) //45 degree
            {
                if(fMag < mag.at<float>(i + 1, j + 1) || fMag < mag.at<float>(i - 1, j - 1))
                {
                    bMaxima = false;
                }
            }
            else if((ang > 67.5 && ang <= 112.5) || ( ang > 247.5 && ang <= 292.5)) // vertical edge
            {
                if(fMag < mag.at<float>(i + 1, j) || fMag < mag.at<float>(i - 1, j))
                {
                    bMaxima = false;
                }
            }
            else if((ang > 112.5 && ang <= 157.5) || ( ang > 292.5 && ang <= 337.5)) // -45 degree
            {
                if(fMag < mag.at<float>(i + 1, j - 1) || fMag < mag.at<float>(i - 1, j + 1))
                {
                    bMaxima = false;
                }
            }

            if(bMaxima == true && fMag > 10)
            {
                nms.at<float>(i, j) = fMag;
                edgeCandidates.push_back(fMag);
            }
        }
}

void CannyEdge::adaptiveThreshold(int &th_low, int &th_high)
{
//    double th = GetThreshVal_Otsu_16u(mag);

//    th_low  = th ;
//    th_high = th * 1.25;


//    if(th_low  < 30) th_low     = 30;
//    if(th_high < 60) th_high    = 60;

//    //  cout << th_high<<endl;
}

void CannyEdge::hysteresisThresholding(int th_low, int th_high)
{
    vector<cv::Point> certainEdges;

    edge = cv::Mat::zeros(nms.size(), CV_8U);

    cv::Mat certain_edge = edge.clone();

    for(int i = 1; i < nms.rows - 1; i++)
        for(int j = 1; j < nms.cols - 1; j++)
        {
            int fMag = nms.at<float>(i, j);

            if(fMag > th_high)
            {
                certain_edge.at<unsigned char>(i, j)    = CERTAIN_EDGE;
                edge.at<unsigned char>(i, j)            = CERTAIN_EDGE;
            }
            else if(fMag > th_low)
            {
                edge.at<unsigned char>(i, j) = PROBABLE_EDGE;
            }
        }

    cv::Mat morphElem(3 , 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(certain_edge, certain_edge, cv::MORPH_DILATE, morphElem);

    for(int i = 1; i < certain_edge.rows - 1; i++)
        for(int j = 1; j < certain_edge.cols - 1; j++)
        {
            if(certain_edge.at<unsigned char>(i, j) != 0)
            {
                certainEdges.push_back(cv::Point(j, i));
            }
        }

    while(!certainEdges.empty())
    {
        cv::Point pt = certainEdges[certainEdges.size() -1];
        certainEdges.pop_back();

        if(edge.at<unsigned char>(pt.y, pt.x + 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y, pt.x + 1) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x + 1, pt.y));
        }

        if(edge.at<unsigned char>(pt.y, pt.x - 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y, pt.x - 1) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x - 1, pt.y ));
        }

        if(edge.at<unsigned char>(pt.y + 1, pt.x) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y + 1, pt.x) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x, pt.y + 1));
        }

        if(edge.at<unsigned char>(pt.y - 1, pt.x) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y - 1, pt.x) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x, pt.y -1));
        }

        if(edge.at<unsigned char>(pt.y + 1, pt.x + 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y + 1, pt.x + 1) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point( pt.x + 1, pt.y + 1));
        }

        if(edge.at<unsigned char>(pt.y + 1, pt.x- 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y + 1, pt.x- 1)= CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x- 1, pt.y + 1));
        }

        if(edge.at<unsigned char>(pt.y - 1, pt.x+ 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y - 1, pt.x+ 1)= CERTAIN_EDGE;
            certainEdges.push_back(cv::Point( pt.x+ 1, pt.y - 1));
        }

        if(edge.at<unsigned char>(pt.y - 1, pt.x- 1) == PROBABLE_EDGE)
        {
            edge.at<unsigned char>(pt.y - 1, pt.x- 1) = CERTAIN_EDGE;
            certainEdges.push_back(cv::Point(pt.x - 1, pt.y- 1));
        }
    }

    for(int i = 0; i < edge.cols * edge.rows; i++)
    {
        if(edge.data[i]!=CERTAIN_EDGE)
        {
            edge.data[i] = 0;
        }
    }
}

void CannyEdge::detection(int th_low, int th_high)
{
    if(th_low == 0 && th_high == 0)
    {
        adaptiveThreshold(th_low, th_high);
    }

    hysteresisThresholding(th_low, th_high);
}

void CannyEdge::makeLine()
{
    const int LineWidth = 60;
    const int LineDistThreshold = 10;

    for(int i= 1; i<edge.rows - 1; i++)
    {
        for(int j=1; j< edge.cols - 1; j++)
        {
            if(edge.at<unsigned char>(i, j) == CERTAIN_EDGE)
            {
                vector<int> vecX;

                for(int dx = 0; dx < LineWidth &&  j + dx < edge.cols -1; dx++)
                {
                    if(edge.at<unsigned char>(i, j + dx) == CERTAIN_EDGE)
                    {
                        vecX.push_back(j + dx);
                    }
                }

                if(vecX.size() > 1)
                {
                    int endX = vecX[vecX.size()-1];
                    int midX = (j + endX)/2;

                    if(endX - j >= LineDistThreshold)
                    {
                        for(int idx = 0; idx < vecX.size(); idx++)
                        {
                            edge.at<unsigned char>(i, vecX[idx]) = 0;
                        }

                        if(midX -1 > 0)
                            edge.at<unsigned char>(i, midX - 1) = CERTAIN_EDGE;

                        edge.at<unsigned char>(i, midX)     = CERTAIN_EDGE;

                        if(midX + 1 < edge.cols - 1)
                            edge.at<unsigned char>(i, midX + 1) = CERTAIN_EDGE;
                    }
                    else
                    {
                        for(int idx = 0; idx < vecX.size(); idx++)
                        {
                            edge.at<unsigned char>(i, vecX[idx]) = 0;
                        }
                    }

                    j = endX + 1;
                }
            }
        }
    }
}

void CannyEdge::makeVerticalLine()
{
    const int LineWidth = 15;
    const int LineDistThreshold = 2;

    for(int i= 1; i<edge.cols - 1; i++)
    {
        for(int j=1; j< edge.rows - 1; j++)
        {
            if(edge.at<unsigned char>(j, i) == CERTAIN_EDGE)
            {
                vector<int> vecY;

                for(int dy = 0; dy < LineWidth &&  j + dy < edge.rows -1; dy++)
                {
                    if(edge.at<unsigned char>(j + dy, i) == CERTAIN_EDGE)
                    {
                        vecY.push_back(j + dy);
                    }
                }

                if(vecY.size() > 1)
                {
                    int endY = vecY[vecY.size()-1];
                    int midY = (j + endY)/2;

                    if(endY - j >= LineDistThreshold)
                    {
                        for(int idx = 0; idx < vecY.size(); idx++)
                        {
                            edge.at<unsigned char>(vecY[idx], i) = 0;
                        }

                        if(midY -1 > 0)
                            edge.at<unsigned char>(midY - 1, i) = CERTAIN_EDGE;

                        edge.at<unsigned char>(midY, i)     = CERTAIN_EDGE;

                        if(midY + 1 < edge.rows - 1)
                            edge.at<unsigned char>(midY + 1, i) = CERTAIN_EDGE;
                    }
                    else
                    {
                        for(int idx = 0; idx < vecY.size(); idx++)
                        {
                            edge.at<unsigned char>(vecY[idx], i) = 0;
                        }
                    }

                    j = endY + 1;
                }
            }
        }
    }
}
