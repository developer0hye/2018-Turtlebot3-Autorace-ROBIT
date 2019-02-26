#include "../include/turtle_vision/trafficlightrecognition.hpp"

using namespace cv;

void TrafficLightRecognition::recognitionBasedYolo(cv::Mat src,int& light)
{
    cv::GaussianBlur(src, src, cv::Size(9, 9), 1.5);

    cv::Mat roiTrafficLight = src;

    cv::Mat srcTrafficLightLab,
            srcTrafficLightLabCH[3],
            foreground;

    cv::Mat srcTrafficLightHsv;
    cv::Mat srcTrafficLightYuv;

    //0:RED 1:YEL 2:GREEN

    cv::Mat morphElem(2, 2, CV_8U, cv::Scalar(1));

    cv::cvtColor(roiTrafficLight, srcTrafficLightYuv, CV_BGR2YUV);
    cv::cvtColor(roiTrafficLight, srcTrafficLightLab, CV_BGR2Lab);
    cv::split(srcTrafficLightLab,srcTrafficLightLabCH);

    cv::threshold( srcTrafficLightLabCH[0], foreground, 50, 255, cv::THRESH_BINARY);

    cv::cvtColor(roiTrafficLight, srcTrafficLightHsv, CV_BGR2HSV);

    //    cv::inRange(srcTrafficLightYuv,
    //                cv::Scalar(0, 0, 160),
    //                cv::Scalar(255, 127 , 255),
    //                trafficLight[0]);

    //        cv::inRange(srcTrafficLightYuv,
    //                    cv::Scalar(0, 0, redLightL[2]),
    //                    cv::Scalar(255, 127 , redLightH[2]),
    //                    trafficLight[0]);


    //    cv::inRange(srcTrafficLightHsv,
    //                cv::Scalar(yellowLightL[0], 25, 0),
    //                cv::Scalar(yellowLightH[0], 255 , 255),
    //                trafficLight[1]);

    //    cv::inRange(srcTrafficLightHsv,
    //                cv::Scalar(greenLightL[0], 25, 0),
    //                cv::Scalar(greenLightH[0], 255 , 255),
    //               trafficLight[2]);


    cv::inRange(srcTrafficLightYuv,
                cv::Scalar(0,0,redLightL[0]),
                cv::Scalar(255,127,redLightH[0]),
                trafficLight[0]);


   /* cv::inRange(srcTrafficLightYuv,
                cv::Scalar(0,0,yellowLightL[0]),
                cv::Scalar(255,127,yellowLightH[0]),
                trafficLight[1]);*/


    cv::inRange(srcTrafficLightHsv,
                cv::Scalar(yellowLightL[0], 65, 50),
                cv::Scalar(yellowLightH[0], 255, 255),
                trafficLight[1]);

    cv::inRange(srcTrafficLightHsv,
                cv::Scalar(greenLightL[0], greenLightL[1], 50),
                cv::Scalar(greenLightH[0], greenLightH[1], 255),
                trafficLight[2]);

    cv::morphologyEx(foreground, foreground, cv::MORPH_DILATE, morphElem,cv::Point());
    for(int i = 0; i < 3; i++) cv::bitwise_and(foreground, trafficLight[i], trafficLight[i]);
    for(int i = 0; i < 3; i++) cv::morphologyEx(trafficLight[i], trafficLight[i], cv::MORPH_CLOSE, morphElem);

    //for(int i = 0; i < 3; i++) cv::morphologyEx(trafficLight[i], trafficLight[i], cv::MORPH_OPEN, morphElem );
   // for(int i = 0; i < 3; i++) cv::morphologyEx(trafficLight[i], trafficLight[i], cv::MORPH_DILATE, morphElem);

    int maxArea     = 0;
    int maxLight    = -1;
    for(int nLight = 0; nLight < 3; nLight++)
    {
        cv::Mat img_labels, stats, centroids;
        int numOfLables = cv::connectedComponentsWithStats(trafficLight[nLight], img_labels, stats, centroids, 8, CV_32S);

        for(int i = 1; i < numOfLables; i++)
        {
            int area    = stats.at<int>(i, cv::CC_STAT_AREA);
            int left    = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top     = stats.at<int>(i, cv::CC_STAT_TOP);
            int width   = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height  = stats.at<int>(i, cv::CC_STAT_HEIGHT);

            double widthHeightRatio = (double)width/height;

            bool bWidthHeightRatioConstraint = ( widthHeightRatio >= (1.0 - widthHeightRatioWeight) && widthHeightRatio <= (1.0 + widthHeightRatioWeight));

            int minArea = trafficLightMinSize;

            if(nLight == 0) minArea = 40;

            bool bAreaConstraint = (area >= minArea && area <= trafficLightMaxSize);

            if(bAreaConstraint)
            {
                if(maxArea < area)
                {
                    maxArea = area;

                    cv::Point center;
                    center.x = left + width/2;
                    center.y = top  + height/2;

//                    if(center.y < roiTrafficLight.rows/3 && nLight == 1) // red
//                    {
//                        maxLight = 0;
//                    }
//                    else
                    {
                        maxLight = nLight;
                    }
                }
            }
            else
            {
                std::cout << "area = "<< area << std::endl;
                std::cout << "widthHeightRatio = "<< widthHeightRatio << std::endl;
            }
        }
    }


    light = maxLight;

    //    cv::imshow("srcinRoi", src);
    //    cv::imshow("srcTrafficLightLabCH[0]", srcTrafficLightLabCH[0]);
    //    cv::imshow("foreground", foreground);

    //    cv::imshow("trafficLight[0]",trafficLight[0]);
    //    cv::imshow("trafficLight[1]",trafficLight[1]);
    //    cv::imshow("trafficLight[2]",trafficLight[2]);
}

void TrafficLightRecognition::recognition(cv::Mat src, cv::Rect& roiTrafficLight, Point &lightCenter, int& light)
{
    cv::Mat filtered_src;

    cv::Mat trafficLightRoi_HSV,
            trafficLightRoi_YUV,
            trafficLightROi_GRAY;

    cv::GaussianBlur(src, filtered_src, cv::Size(7,7), 1.0);

    cv::cvtColor(filtered_src, trafficLightRoi_HSV, CV_BGR2HSV);
    cv::cvtColor(filtered_src, trafficLightRoi_YUV, CV_BGR2YUV);
    cv::cvtColor(filtered_src, trafficLightROi_GRAY, CV_BGR2GRAY);

    cv::Mat foreground;
    cv::Mat trafficLightBlackBody;

    cv::Mat trafficLight[3]; //0:RED 1:YEL 2:GREEN

    cv::Mat morphElemOpen(4, 4, CV_8U, cv::Scalar(1));

    cv::inRange(trafficLightRoi_HSV, cv::Scalar(0, 50, 25), cv::Scalar(180, 255 , 255), foreground);

    cv::inRange(trafficLightRoi_YUV, cv::Scalar(0, 0, 200), cv::Scalar(255, 127 , 255), trafficLight[0]);
    cv::bitwise_and(foreground, trafficLight[0], trafficLight[0]);

    cv::inRange(trafficLightRoi_YUV, cv::Scalar(0, 0, 153), cv::Scalar(255, 127  , 200), trafficLight[1]);
    cv::bitwise_and(foreground, trafficLight[1], trafficLight[1]);

    cv::inRange(trafficLightRoi_HSV, cv::Scalar(70, 50, 50), cv::Scalar(95, 255 , 255), trafficLight[2]);

    cv::threshold( trafficLightROi_GRAY, trafficLightBlackBody, 127, 255, cv::THRESH_BINARY_INV);

    for(int i = 0; i < 3; i++) cv::morphologyEx(trafficLight[i], trafficLight[i], cv::MORPH_OPEN, morphElemOpen );
    for(int i = 0; i < 3; i++) cv::morphologyEx(trafficLight[i], trafficLight[i], cv::MORPH_DILATE, morphElemOpen );

    cv::morphologyEx(trafficLightBlackBody, trafficLightBlackBody, cv::MORPH_DILATE, morphElemOpen);

    for(int i = 0; i < 3; i++) cv::subtract(trafficLightBlackBody, trafficLight[i], trafficLightBlackBody);

    int numOfLablesTrafficLight[3] = {0,};
    int maxLightSize = 0;

    for(int nLight = 0; nLight < 3; nLight++)
    {
        cv::Mat img_labels, stats, centroids;
        numOfLablesTrafficLight[nLight] = cv::connectedComponentsWithStats(trafficLight[nLight], img_labels, stats, centroids, 8, CV_32S);

        for(int i = 1; i < numOfLablesTrafficLight[nLight]; i++)
        {
            double area = stats.at<int>(i, cv::CC_STAT_AREA);
            double left = stats.at<int>(i, cv::CC_STAT_LEFT);
            double top = stats.at<int>(i, cv::CC_STAT_TOP);
            double width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            double height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            double size = width * height;

            double ratioWhite = (double)area/size;
            double lengthRatio = width/height;

            cv::Point center;

            center.x = left + width/2;
            center.y = top  + height/2;

            bool bSizeConstraint = (area >= trafficLightMinSize && area <= trafficLightMaxSize);
            bool bLabeldRatioConstraint = ratioWhite >= labledRatio ;
            bool bWidthHeightRatioConstraint = ( lengthRatio >= (1.0 - widthHeightRatioWeight) && lengthRatio <= (1.0 + widthHeightRatioWeight));
            bool bPosTrafficLight = center.x < (double)trafficLightBlackBody.cols * 0.75;

            if( bSizeConstraint && bLabeldRatioConstraint && bWidthHeightRatioConstraint)
            {
                cv::Point startTrafficLight, endTrafficLight;

                switch (nLight)
                {
                case 0: //red
                    startTrafficLight.x = left - width*0.5;
                    startTrafficLight.y = top - height * 0.5;

                    endTrafficLight.x = left + width*1.5;
                    endTrafficLight.y = top  + height*5.0;

                    //  cv::rectangle(src, cv::Rect(startTrafficLight, endTrafficLight), cv::Scalar(0,0,255),2);
                    break;
                case 1: //yellow
                    startTrafficLight.x = left - width*0.5;
                    startTrafficLight.y = top - height * 2.5;

                    endTrafficLight.x = left + width*1.5;
                    endTrafficLight.y = top  + height*4.0;

                    //  cv::rectangle(src, cv::Rect(startTrafficLight, endTrafficLight), cv::Scalar(0,255,255),2);

                    break;
                case 2: //green
                    startTrafficLight.x = left - width*0.5;
                    startTrafficLight.y = top - height * 3.5;

                    endTrafficLight.x = left + width*1.5;
                    endTrafficLight.y = top + height*2.0;


                    // cv::rectangle(src, cv::Rect(startTrafficLight, endTrafficLight), cv::Scalar(0,255,0),2);
                    break;

                default:
                    break;
                }

                if(startTrafficLight.x < 0) startTrafficLight.x = 0;
                if(startTrafficLight.y < 0) startTrafficLight.y = 0;
                if(startTrafficLight.x >= src.cols) startTrafficLight.x = src.cols -1;
                if(startTrafficLight.y >= src.rows) startTrafficLight.y = src.rows -1;

                if(endTrafficLight.x < 0) endTrafficLight.x = 0;
                if(endTrafficLight.y < 0) endTrafficLight.y = 0;
                if(endTrafficLight.x >= src.cols) endTrafficLight.x = src.cols -1;
                if(endTrafficLight.y >= src.rows) endTrafficLight.y = src.rows -1;

                cv::Rect trafficLightRoi = cv::Rect(startTrafficLight, endTrafficLight);


                cv::Mat roiBlack = trafficLightBlackBody(trafficLightRoi);

                int nForeground = 0;
                for(int y = 0; y < roiBlack.rows; y++)
                    for(int x = 0; x < roiBlack.cols; x++)
                    {
                        if(roiBlack.at<uchar>(y, x) == 255)
                        {
                            nForeground++;
                        }
                    }

                double foregroundRatio = (double)nForeground/(roiBlack.rows*roiBlack.cols);

                //                std::cout<<"foregroundRatio: "<<foregroundRatio<< std::endl;

                if(foregroundRatio >= 0.88)
                {
                    if(maxLightSize < size)
                    {
                        maxLightSize = size;

                        switch (nLight)
                        {
                        case 0: //red

                            cv::rectangle(src, trafficLightRoi, cv::Scalar(0,0,255),2);

                            break;
                        case 1: //yellow

                            cv::rectangle(src, trafficLightRoi, cv::Scalar(0,255,255),2);

                            break;
                        case 2: //green

                            cv::rectangle(src, trafficLightRoi, cv::Scalar(0,255,0),2);
                            break;

                        default:
                            break;
                        }

                        light = nLight;
                        roiTrafficLight = trafficLightRoi;

                        lightCenter = center;
                    }

                }
            }
            else
            {
                //                std::cout<<"area"<< area <<std::endl;
                //                std::cout<<"lengthRatio"<< lengthRatio <<std::endl;
                //                std::cout<<"bSizeConstraint"<< bSizeConstraint <<std::endl;
                //                std::cout<<"bLabeldRatioConstraint"<< bLabeldRatioConstraint <<std::endl;
                //                std::cout<<"bWidthHeightRatioConstraint"<< bWidthHeightRatioConstraint <<std::endl;
            }
        }
    }
}
