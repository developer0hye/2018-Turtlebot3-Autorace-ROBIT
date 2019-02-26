#include "../include/turtle_vision/lifevision.hpp"


#define     POINT_0_X   30
#define     POINT_0_Y   50

#define     POINT_1_X   290
#define     POINT_1_Y   50

#define     POINT_2_X   320
#define     POINT_2_Y   80

#define     POINT_3_X   0
#define     POINT_3_Y   80

#define     POINT_PARKING_0_X   120 + 30
#define     POINT_PARKING_0_Y   50

#define     POINT_PARKING_1_X   290
#define     POINT_PARKING_1_Y   50

#define     POINT_PARKING_2_X   320
#define     POINT_PARKING_2_Y   80

#define     POINT_PARKING_3_X   120 + 60
#define     POINT_PARKING_3_Y   80

#define     MARGIN_OF_LIGHT 20


#define     LINE_RANSAC_MODE 1


#define trafficLightDebugging 0

const int startTrafficROiX = 192;
const int startTrafficROiY = 40;
const int startTrafficROiWidth = 320 - startTrafficROiX;
const int startTrafficROiHeight = 128;

using namespace std;
using namespace cv;
using namespace yh;

void detect_mat(Mat frame_detect, float* detections_output, int* num_output_class, float thresh, float hier_thresh, std::vector<cv::Rect>& objBoxes, vector<int>& objLabels)
{
    // do detect in an unsigned char* data buffer getted from Mat::data or IplImage.imageData
    unsigned char* data = (unsigned char*)frame_detect.data;
    int w = frame_detect.cols;
    int h = frame_detect.rows;
    int c = frame_detect.channels();
    float* detections = test_detector_uchar(data, w, h, c, thresh, hier_thresh, num_output_class);
    for(int i = 0; i < *num_output_class; i++)
    {
        detections_output[i*6+0] = detections[i*6+0];// ith detection's category
        detections_output[i*6+1] = detections[i*6+1];// ith detection's confidence score
        detections_output[i*6+2] = detections[i*6+2];// ith detection's top-left x-coordinate of bbox
        detections_output[i*6+3] = detections[i*6+3];// ith detection's top-left y-coordinate of bbox
        detections_output[i*6+4] = detections[i*6+4];// ith detection's width of bbox
        detections_output[i*6+5] = detections[i*6+5];// ith detection's height of bbox

        if(detections[i*6+4] < frame_detect.cols * 0.01) continue;
        if(detections[i*6+5] < frame_detect.rows * 0.01) continue;


        objBoxes.push_back(cv::Rect(detections_output[i*6+2], detections_output[i*6+3], detections_output[i*6+4], detections_output[i*6+5]));
        objLabels.push_back(detections_output[i*6+0]);
    }
}


void transform(cv::Point2f* src_vertices, cv::Point2f* dst_vertices, cv::Mat& src, cv::Mat &dst)
{
    cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(src, dst, M, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void MakeHorizontalOneLine(cv::Mat& src, cv::Mat& dst, int minLineWidth = 10, int maxLineWidth = 45)
{
    if(dst.empty()) dst = cv::Mat::zeros(src.size(),CV_8UC1);

    for(int y = 1; y < src.rows -1; y++)
    {
        bool edgeStarted = false;

        int lineStartX  = 0;
        int lineWidth = 0;

        for(int x = 1; x < src.cols-1; x++)
        {
            if(edgeStarted == false && src.at<uchar>(y,x) != 0)
            {
                lineStartX = x;
                edgeStarted = true;
            }
            else if(edgeStarted == true)
            {
                lineWidth = x - lineStartX;

                if(lineWidth > maxLineWidth)
                {
                    edgeStarted = false;
                }
                else
                {
                    if( lineWidth >= minLineWidth && src.at<uchar>(y,x) != 0)
                    {
                        dst.at<uchar>(y,lineStartX + lineWidth/2) = 255;
                        edgeStarted = false;
                    }
                }
            }
        }
    }
}

void MakeVerticalOneLine(cv::Mat& src, cv::Mat& dst, int minLineWidth = 10, int maxLineWidth = 45)
{
    if(dst.empty()) dst = cv::Mat::zeros(src.size(),CV_8UC1);

    for(int x = 1; x < src.cols-1; x++)
    {
        bool edgeStarted = false;

        int lineStartY  = 0;
        int lineWidth = 0;

        for(int y = 1; y < src.rows -1; y++)
        {
            if(edgeStarted == false && src.at<uchar>(y,x) != 0)
            {
                lineStartY = y;
                edgeStarted = true;
                lineWidth = 1;
            }
            else if(edgeStarted == true)
            {
                lineWidth = y - lineStartY;

                if(lineWidth >= maxLineWidth)
                {
                    edgeStarted = false;
                }
                else
                {
                    if( lineWidth >= minLineWidth && src.at<uchar>(y,x) != 0)
                    {
                        dst.at<uchar>(lineStartY + lineWidth/2, x) = 255;
                        edgeStarted = false;
                    }
                }
            }
        }
    }
}

void removeSmallEdgeComponents(cv::Mat& src, int minEdgeComponentArea = 15)
{
    cv::Mat img_labels, stats, centroids;
    int numOfLables = cv::connectedComponentsWithStats(src, img_labels,
                                                       stats, centroids, 8, CV_32S);

    //라벨링 된 이미지에 각각 직사각형으로 둘러싸기
    for (int j = 1; j < numOfLables; j++)
    {
        int area = stats.at<int>(j, cv::CC_STAT_AREA);
        int left = stats.at<int>(j, cv::CC_STAT_LEFT);
        int top = stats.at<int>(j, cv::CC_STAT_TOP);
        int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(j, cv::CC_STAT_HEIGHT);

        if(area <= minEdgeComponentArea)
        {
            for(int y = top; y < top + height; y++)
                for(int x = left; x < left + width; x++)
                {
                    if(img_labels.at<int>(y, x) == j)
                    {
                        src.at<uchar>(y,x) = 0;
                    }
                }
        }
    }
}

LifeVision::LifeVision(QObject *parent) :
    QObject(parent),
    parking_mode(false),
    parking_mark_cnt(0),
    parking_line_flag(false),
    parking_line_cnt(1),
    parking_line_start(false),
    across_mark_cnt(0),
    across_flag(false),
    victory_data(0)
{
    src_vertices[0] = cv::Point(POINT_0_X, POINT_0_Y);
    src_vertices[1] = cv::Point(POINT_1_X, POINT_1_Y);
    src_vertices[2] = cv::Point(POINT_2_X, POINT_2_Y);
    src_vertices[3] = cv::Point(POINT_3_X, POINT_3_Y);

    dst_vertices[0] = cv::Point(0,0);
    dst_vertices[1] = cv::Point(320,0);
    dst_vertices[2] = cv::Point(320,120);
    dst_vertices[3] = cv::Point(0,120);

    //    detector_init(    "/home/turtle1/Libs/darknet_AlexeyAB/across/yolov3-tiny-across_test.cfg",
    //                      "/home/turtle1/Libs/darknet_AlexeyAB/across/yolov3-tiny-across_4500.weights" );

    detector_init(    "/home/turtle1/Libs/darknet_AlexeyAB/across/yolov3-tiny-across_test.cfg",
                      "/home/turtle1/Libs/darknet_AlexeyAB/across/yolov3-tiny-across_4500.weights" );


    //cv::Mat parkingSignImage = cv::imread("/home/turtle1/catkin_ws/src/turtle_vision/turtle_vision/mark_image/parking_robotis.png");
    cv::Mat parkingSignImage = cv::imread("/home/turtle1/catkin_ws/src/turtle_vision/turtle_vision/mark_image/tunnel.png");
    cv::resize(parkingSignImage, parkingSignImage, cv::Size(50,50));
    siftParking = SiftUsingKNN(parkingSignImage);
}

LifeVision::~LifeVision()
{
    detector_uninit();
}

void LifeVision::process(cv::Mat& src)
{
    srcDrawing = src.clone();

    //msg init()
    vision_msg.signal = 0;

    // trafficLightProcessing
     _trafficLightProcess(src);

    // parkingMarkProcessing
    _parkingProcess(src);

    // acrossMarkProcessing
    _acrossProcess(src);

    // lineProcessing
    _lineProcess(src);

    // pogbaProcessing
    _pogbaProcess(src);

    _drawHelpInform(srcDrawing);

    Q_EMIT updateImg(srcDrawing);
}

void LifeVision::_lineProcess(cv::Mat& src)
{
    cv::Rect roiLine(0, src.rows/2, src.cols, src.rows/2);

    cv::Mat srcLineRoi = src(roiLine).clone();

    // bird view
    cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::Mat dst(srcLineRoi.rows, srcLineRoi.cols, CV_8UC3);
    cv::warpPerspective(srcLineRoi, dst, M, dst.size(), cv::INTER_LINEAR);

    cv::Mat dstTopView(srcLineRoi.rows, srcLineRoi.cols, CV_8UC3);
    cv::Mat dstTopViewGray;

    transform(src_vertices, dst_vertices, srcLineRoi, dstTopView);

   /* yh::CannyEdge cannyEdge(dstTopView);
    cannyEdge.detection(60,90);
    cannyEdge.makeLine();

    cv::Mat morphDilate(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(cannyEdge.edge, cannyEdge.edge, cv::MORPH_DILATE, morphDilate);*/


    cv::GaussianBlur(dstTopView, dstTopView, cv::Size(7, 7), 1.0);
    cv::medianBlur(dstTopView, dstTopView,5);
    cv::cvtColor(dstTopView, dstTopViewGray, CV_BGR2GRAY);

    cv::Mat lineEdge,
            twoLineToOneLineEdgeHorizontal = cv::Mat::zeros(dstTopView.size(), CV_8UC1),
            twoLineToOneLineEdgeVertical = cv::Mat::zeros(dstTopView.size(), CV_8UC1),
            twoLineToOneLineEdgeSum;


    cv::Canny(dstTopViewGray, lineEdge, 60, 120);

    MakeHorizontalOneLine(lineEdge, twoLineToOneLineEdgeHorizontal);
    MakeVerticalOneLine(lineEdge, twoLineToOneLineEdgeVertical);

    cv::Mat morphDilate(3, 3, CV_8U, cv::Scalar(1));
    cv::Mat morphDilateVer(2, 3, CV_8U, cv::Scalar(1));
    cv::Mat morphDilateHor(3, 2, CV_8U, cv::Scalar(1));

    cv::morphologyEx(twoLineToOneLineEdgeHorizontal, twoLineToOneLineEdgeHorizontal, cv::MORPH_DILATE, morphDilateHor);
    cv::morphologyEx(twoLineToOneLineEdgeVertical, twoLineToOneLineEdgeVertical, cv::MORPH_DILATE, morphDilateVer);

    removeSmallEdgeComponents(twoLineToOneLineEdgeHorizontal, 15);
    removeSmallEdgeComponents(twoLineToOneLineEdgeVertical, 15);

    twoLineToOneLineEdgeSum = twoLineToOneLineEdgeHorizontal + twoLineToOneLineEdgeVertical;
    cv::morphologyEx(twoLineToOneLineEdgeSum, twoLineToOneLineEdgeSum, cv::MORPH_DILATE, morphDilate);

    removeSmallEdgeComponents(twoLineToOneLineEdgeSum, 30);

    cv::Mat leftLine, rightLine;
    __lineJudge(dstTopView, twoLineToOneLineEdgeSum, leftLine, rightLine, parking_mode);
    //__lineJudge(dstTopView, cannyEdge.edge, leftLine, rightLine, parking_mode);

    // line extraction
    __lineExtraction(dstTopView, leftLine, rightLine);

    cv::rectangle(srcDrawing, cv::Rect(0, src.rows - src.rows/6, src.cols, src.rows/6), cv::Scalar(255,255,0),1);

    if(parking_mode == true && vision_msg.right_line_degree == 0.0 && vision_msg.left_line_degree == 0.0)
    {
        std::cout <<"parking mode true" << std::endl;
        cv::Mat lineRoi = src(cv::Rect(0, src.rows - src.rows/6, src.cols, src.rows/6)).clone();

        cv::Mat edge, whiteEdge = cv::Mat::zeros(lineRoi.size(), CV_8UC1);
        cv::Canny(lineRoi, edge, 60, 120);

        cv::Mat morphDilate(3, 3, CV_8U, cv::Scalar(1));
        cv::morphologyEx(edge, edge, cv::MORPH_DILATE, morphDilate);

        cv::Mat srcHsv;
        cv::Mat frameMaskYellow;

        cv::cvtColor(lineRoi, srcHsv, CV_BGR2HSV);

        cv::inRange(srcHsv, cv::Scalar(20, 35, 50), cv::Scalar(45, 255, 255), frameMaskYellow);

        cv::Mat morphElem(3, 3, CV_8U, cv::Scalar(1));

        cv::dilate(frameMaskYellow, frameMaskYellow, morphElem,cv::Point());

        int kernelSize = 1;
        for(int i = kernelSize; i < edge.rows - kernelSize; i++)
        {
            for(int j = kernelSize; j < edge.cols - kernelSize; j++)
            {
                if(edge.at<uchar>(i, j) != 0)
                {
                    int nYellow = 0;

                    for(int dy = -kernelSize; dy <= kernelSize; dy++)
                        for(int dx = -kernelSize; dx <= kernelSize; dx++)
                        {
                            if(frameMaskYellow.at<uchar>(i + dy, j + dx) != 0)
                                nYellow++;
                        }

                    if(nYellow >= 1)
                    {
                        // yellowEdge.at<uchar>(i, j) = 255;
                    }
                    else
                    {
                        whiteEdge.at<uchar>(i, j) = 255;
                    }
                }
            }
        }

    //    cv::Mat morphDilate(3, 3, CV_8U, cv::Scalar(1));

        cv::Mat morphVerDilate(2, 2, CV_8U, cv::Scalar(1));
        cv::morphologyEx(edge, edge, cv::MORPH_DILATE, morphVerDilate);


        vector<Vec4i> lines;
        HoughLinesP(edge, lines, 1, CV_PI/180., edge.cols/5., edge.cols/5.);

        std::cout<<"lines.size() = " << lines.size() << std::endl;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];

            cv::Point p1(l[0], l[1]);
            cv::Point p2(l[2], l[3]);

            if(p1.x < p2.x)
            {
                cv::Point temp_point = p1;
                p1 = p2;
                p2 = temp_point;
            }

            double temp_degree=-(atan2(p1.y - p2.y, p1.x - p2.x)*180.0)/CV_PI;
            if(temp_degree<0.0)temp_degree+=180.0;

            std::cout << "temp_degree : " << temp_degree << std::endl;
            if(temp_degree <= 10.0 || temp_degree >= 170.0)
            {
                std::cout <<"horizontal line detected!"<< std::endl;
                line( lineRoi, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
                vision_msg.right_line_degree = -1.0;
                vision_msg.left_line_degree  = -1.0;
            }
        }

        Q_EMIT updateLineImg(lineRoi);
    }
    else
        Q_EMIT updateLineImg(dstTopView);

}

void LifeVision::__lineExtraction(cv::Mat& src, cv::Mat& leftLine, cv::Mat& rightLine)
{
#if LINE_RANSAC_MODE
    JYJ_RansacLine leftLineRansac(leftLine, true, false),
            rightLineRansac(rightLine, true, true);

    leftLineRansac.runRansac();
    rightLineRansac.runRansac();

    cv::line(src, leftLineRansac.getLineEnd(), leftLineRansac.getLineStart(), cv::Scalar(255,0,0),3);
    cv::line(src, rightLineRansac.getLineEnd(), rightLineRansac.getLineStart(), cv::Scalar(255,0,255),3);

    vision_msg.right_line_degree    =   rightLineRansac.getLineDegree();
    vision_msg.left_line_degree     =   leftLineRansac.getLineDegree();
    vision_msg.right_line_start_X   =   rightLineRansac.getLineStart().x;
    vision_msg.right_line_start_Y   =   rightLineRansac.getLineStart().y;
    vision_msg.right_line_end_X     =   rightLineRansac.getLineEnd().x;
    vision_msg.right_line_end_Y     =   rightLineRansac.getLineEnd().y;
    vision_msg.left_line_start_X    =   leftLineRansac.getLineStart().x;
    vision_msg.left_line_start_Y    =   leftLineRansac.getLineStart().y;
    vision_msg.left_line_end_X      =   leftLineRansac.getLineEnd().x;
    vision_msg.left_line_end_Y      =   leftLineRansac.getLineEnd().y;

#else
    vector<cv::Vec4i> leftLines;
    cv::HoughLinesP(leftLine,
                    leftLines,
                    1,
                    CV_PI/180,
                    leftLine.rows/3,
                    leftLine.rows/2,
                    10 );

    double maxLeftLineLength = 0.0;
    cv::Point leftP1(-1, -1), leftP2(0,0);

    for( size_t i = 0; i < leftLines.size(); i++ )
    {
        cv::Vec4i l = leftLines[i];

        double lineLength = cv::norm(cv::Point(l[0], l[1]) - cv::Point(l[2], l[3]));

        if(maxLeftLineLength < lineLength)
        {
            maxLeftLineLength = lineLength;
            leftP1 = cv::Point(l[0], l[1]);
            leftP2 = cv::Point(l[2], l[3]);
        }
    }

    if(leftP1 != cv::Point(-1, -1))
    {
        if(leftP1.x < leftP2.x)
        {
            cv::swap(leftP1, leftP2);
        }

        double temp_degree=-(atan2(leftP1.y - leftP2.y, leftP1.x - leftP2.x)*180.0)/CV_PI;
        if(temp_degree < 0.0)temp_degree+=180.0;

        if(temp_degree < 15.0 || temp_degree > 115.0 )
        {
            return ;
        }
        else
        {
            std::cout << temp_degree <<  std::endl;

            if(leftP1.y < leftP2.y)
            {
                cv::swap(leftP1, leftP2);
            }

            vision_msg.left_line_degree     =   temp_degree;
            vision_msg.left_line_start_X    =   leftP1.x;
            vision_msg.left_line_start_Y    =   leftP1.y;
            vision_msg.left_line_end_X      =   leftP2.x;
            vision_msg.left_line_end_Y      =   leftP2.y;

            cv::line( src, leftP1, leftP2, cv::Scalar(0,0,255), 3, CV_AA);
        }
    }
    else
    {
        vision_msg.left_line_degree     =   0;
        vision_msg.left_line_start_X    =   0;
        vision_msg.left_line_start_Y    =   0;
        vision_msg.left_line_end_X      =   0;
        vision_msg.left_line_end_Y      =   0;
    }

    vector<cv::Vec4i> rightLines;
    cv::HoughLinesP(rightLine,
                    rightLines,
                    1,
                    CV_PI/180,
                    rightLine.rows/3,
                    rightLine.rows/2,
                    10 );

    double maxrightLineLength = 0.0;
    cv::Point rightP1(-1, -1), rightP2(0,0);

    for( size_t i = 0; i < rightLines.size(); i++ )
    {
        cv::Vec4i l = rightLines[i];

        double lineLength = cv::norm(cv::Point(l[0], l[1]) - cv::Point(l[2], l[3]));

        if(maxrightLineLength < lineLength)
        {
            maxrightLineLength = lineLength;
            rightP1 = cv::Point(l[0], l[1]);
            rightP2 = cv::Point(l[2], l[3]);
        }
    }

    if(rightP1 != cv::Point(-1, -1))
    {
        if(rightP1.x < rightP2.x)
        {
            cv::swap(rightP1, rightP2);
        }

        double temp_degree=-(atan2(rightP1.y - rightP2.y, rightP1.x - rightP2.x)*180.0)/CV_PI;
        if(temp_degree < 0.0)temp_degree+=180.0;

        if(temp_degree < 85.0 || temp_degree > 165.0 )
        {
            return ;
        }
        else
        {
            std::cout << temp_degree <<  std::endl;

            if(rightP1.y < rightP2.y)
            {
                cv::swap(rightP1, rightP2);
            }

            vision_msg.right_line_degree     =   temp_degree;
            vision_msg.right_line_start_X    =   rightP1.x;
            vision_msg.right_line_start_Y    =   rightP1.y;
            vision_msg.right_line_end_X      =   rightP2.x;
            vision_msg.right_line_end_Y      =   rightP2.y;

            cv::line( src, rightP1, rightP2, cv::Scalar(255,0,255), 3, CV_AA);
        }
    }
    else
    {
        vision_msg.right_line_degree     =   0;
        vision_msg.right_line_start_X    =   0;
        vision_msg.right_line_start_Y    =   0;
        vision_msg.right_line_end_X      =   0;
        vision_msg.right_line_end_Y      =   0;
    }

#endif
}

void LifeVision::_trafficLightProcess(cv::Mat &src)
{
    std::cout<<"---trafficLightProcess---" << std::endl;
    cv::Rect trafficLightROI(242, 100, 25, 20);

    cv::Mat srcTrafficLightROI = src(trafficLightROI).clone();
    cv::GaussianBlur(srcTrafficLightROI, srcTrafficLightROI, cv::Size(7,7), 1.0);

    cv::Mat trafficLightHSV, light;

    cv::cvtColor(srcTrafficLightROI, trafficLightHSV, CV_BGR2HSV);
    cv::rectangle(srcDrawing, trafficLightROI, cv::Scalar(0,255,0), 1);

    cv::inRange(trafficLightHSV, cv::Scalar(50, 0, 150), cv::Scalar(100, 255, 255), light);

    cv::Mat morphElem(3, 3, CV_8U, cv::Scalar(1));
    cv::morphologyEx(light, light, cv::MORPH_CLOSE, morphElem,cv::Point());

    cv::Mat img_labels, stats, centroids;
    int numOfLables = cv::connectedComponentsWithStats(light, img_labels,
                                                       stats, centroids, 8, CV_32S);
    //라벨링 된 이미지에 각각 직사각형으로 둘러싸기
    for (int j = 1; j < numOfLables; j++)
    {
        int area = stats.at<int>(j, cv::CC_STAT_AREA);

        int left = stats.at<int>(j, cv::CC_STAT_LEFT);
        int top = stats.at<int>(j, cv::CC_STAT_TOP);

        int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(j, cv::CC_STAT_HEIGHT);

        double foregroundRatio  = (double)area/(width*height);
        double aspectRatio      = (double)width/height;

        bool bAreaThreshold     = (area >= 50);
        bool bForegroundRatio   = (foregroundRatio >= 0.65);
        bool bAspectRatio       = (aspectRatio <= 1.5 && aspectRatio >= 0.5);
        bool bAllConditions     = (bAreaThreshold && bForegroundRatio && bAspectRatio);


#if trafficLightDebugging
        std::cout <<"trafficLight area:" << area;
        if(!bAreaThreshold) std::cout << " area too small (area < 30)";
        std::cout << std::endl;

        std::cout <<"trafficLight foreground Ratio:" << foregroundRatio;
        if(!bForegroundRatio) std::cout <<" foreground ratio too low (foregroundRatio < 0.65)";
        std::cout << std::endl;

        std::cout <<"trafficLight aspectRatio:" << aspectRatio;
        if(!bAspectRatio) std::cout <<" aspect ratio too low (aspectRatio > 1.5 or aspectRatio < 0.5);";
        std::cout << std::endl;
#endif
        if(bAllConditions)
        {
            std::cout <<"trafficLight Detected!!" << std::endl;
            vision_msg.signal = GREEN_LIGHT;
        }
    }

    Q_EMIT updateTrafficLightGreenImg(light);

    std::cout<<"---End trafficLightProcess---" << std::endl;

    //    static cv::Rect roiTrafficLight(0,
    //                             0,
    //                             320,
    //                             120);

    //    cv::Mat frameTrafficLight = src(roiTrafficLight).clone();

    //    float *detections = (float*)calloc(255*6, sizeof(float));
    //    int num_output_class = 0;
    //    double threshold = .5;
    //    double hier_thresh = .0;
    //    vector<cv::Rect> results;

    //    detect_mat(frameTrafficLight,
    //               detections,
    //               &num_output_class,
    //               threshold,
    //               hier_thresh,
    //               results);

    //    for(int i = 0; i < results.size(); i++)
    //    {
    //        results[i].x += roiTrafficLight.x;
    //                results[i].y += roiTrafficLight.y;
    //        cv::rectangle(src, results[i], cv::Scalar(0,255,0));
    //    }

//    static cv::Rect roiTrafficLight(startTrafficROiX,
//                                    startTrafficROiY,
//                                    startTrafficROiWidth,
//                                    startTrafficROiHeight);

//    cv::Mat frameTrafficLight = src(roiTrafficLight).clone();

//    float *detections = (float*)calloc(255*6, sizeof(float));
//    int num_output_class = 0;
//    double threshold = .5;
//    double hier_thresh = .0;
//    vector<cv::Rect> results;
//    vector<int> labelResults;

//    detect_mat(frameTrafficLight,
//               detections,
//               &num_output_class,
//               threshold,
//               hier_thresh,
//               results, labelResults);


//    img_line_edge = frameTrafficLight.clone();

//    cv::Rect trafficLightROI(0,0,0,0);

//    static int          past_light_signal = -1;
//    static cv::Point    trafficLightCricleCenter(0,0);
//    static int          notfoundTrafficLight = 0;

//    if(!results.empty())
//    {
//        notfoundTrafficLight = 0;

//        int maxArea = 0;
//        for(int i = 0; i < num_output_class; i++)
//        {
//            if(results[i].height < results[i].width) continue;

//            if(maxArea < results[i].height * results[i].width )
//            {
//                maxArea = results[i].height * results[i].width;

//                trafficLightROI = results[i];
//                trafficLightROI.x += roiTrafficLight.x;
//                trafficLightROI.y += roiTrafficLight.y;
//            }
//        }
//    }

//    free(detections);
//    TrafficLightRecognition tr;
//    bool bFoundTrafficLight = false;

//    if(trafficLightROI.width != 0)
//    {
//        tr.setGreenLight(cv::Scalar(traffic_light_green_hue_min, traffic_light_green_sat_min, 25),
//                         cv::Scalar(traffic_light_green_hue_max, traffic_light_green_sat_max, 255));
//        tr.setRedLight(cv::Scalar(traffic_light_red_hue_min, traffic_light_red_sat_min, 25),
//                       cv::Scalar(traffic_light_red_hue_max, traffic_light_red_sat_max, 255));
//        tr.setYellowLight(cv::Scalar(traffic_light_yellow_hue_min, traffic_light_yellow_sat_min, 25),
//                          cv::Scalar(traffic_light_yellow_hue_max, traffic_light_yellow_sat_max, 255));

//        int nLight = -1;
//        tr.recognitionBasedYolo(src(cv::Rect(trafficLightROI.x, trafficLightROI.y, trafficLightROI.width*0.9, trafficLightROI.height)).clone() ,nLight);

//        cv::Scalar color;

//        switch (nLight)
//        {
//        case 0: //R

//            //            color = cv::Scalar(0, 0, 255);

//            //            vision_msg.signal = RED_LIGHT;
//            //            past_light_signal = RED_LIGHT;
//            //            vision_msg.red_light_x = (trafficLightROI.x + trafficLightROI.width / 2 )- roiTrafficLight.x;
//            //            bFoundTrafficLight = true;
//            //            trafficLightCricleCenter.x = vision_msg.red_light_x;
//            break;
//        case 1: //YE

//            color = cv::Scalar(0, 255, 255);

//            //            vision_msg.signal = YELLOW_LIGHT;
//            //            past_light_signal = YELLOW_LIGHT;
//            //            vision_msg.yellow_light_x = (trafficLightROI.x + trafficLightROI.width / 2) - roiTrafficLight.x;
//            //            bFoundTrafficLight = true;
//            //            trafficLightCricleCenter.x = vision_msg.yellow_light_x;
//            break;

//        case 2: //GR
//            vision_msg.signal = GREEN_LIGHT;
//            color = cv::Scalar(0, 255, 0);
//            bFoundTrafficLight = true;
//            past_light_signal = 0;
//            break;

//        default:

//            break;
//        }
//        cv::rectangle(src, trafficLightROI, color, 3);
//    }

//    Q_EMIT updateTrafficLightRedImg(tr.trafficLight[0]);
//    Q_EMIT updateTrafficLightYellowImg(tr.trafficLight[1]);
//    Q_EMIT updateTrafficLightGreenImg(tr.trafficLight[2]);
}

void LifeVision::_pogbaProcess(cv::Mat &src)
{
    cv::Rect roiPaulPogba(0, 40, 320, 80);

    cv::Mat framePogba = src(roiPaulPogba).clone();
    cv::Mat frameHsv;
    //cv::Mat frameHsv

    cv::GaussianBlur(framePogba, framePogba, cv::Size(9,9), 1.5);

    cv::cvtColor(framePogba, frameHsv, cv::COLOR_BGR2HSV);

    cv::Mat framePogbaRedL, framePogbaRedH, framePogbaRedSum;

    cv::inRange(frameHsv, cv::Scalar(0, 25, 25), cv::Scalar(15, 255, 255), framePogbaRedL);
    cv::inRange(frameHsv, cv::Scalar(165, 25, 25), cv::Scalar(180, 255, 255), framePogbaRedH);
    //cv::inRange(frameHsv, cv::Scalar(0, pogba_sat_min, pogba_val_min), cv::Scalar(15, pogba_sat_m
    cv::add(framePogbaRedL, framePogbaRedH, framePogbaRedSum);

    cv::Mat morphOpen(15, 15, CV_8U, cv::Scalar(1));
    cv::morphologyEx(framePogbaRedSum, framePogbaRedSum, cv::MORPH_OPEN, morphOpen);

    //    cv::Mat morphElemErode(17, 17, CV_8U, cv::Scalar(1));
    //    cv::Mat morphElemDilate(5, 5, CV_8U, cv::Scalar(1));

    //    cv::morphologyEx(framePogbaRedSum, framePogbaRedSum, cv::MORPH_ERODE, morphElemErode);
    //    cv::morphologyEx(framePogbaRedSum, framePogbaRedSum, cv::MORPH_DILATE, morphElemDilate);

    //    img_line_edge = framePogbaRedSum.clone();


    cv::Mat img_labels,stats, centroids;
    int numOfLables = cv::connectedComponentsWithStats(framePogbaRedSum, img_labels, stats, centroids, 4, CV_32S);

    cv::Point midPoints[2];
    int maxArea_2 = 0;
    int maxArea_1 = 0;


    std::vector<cv::Rect> pogboxCandidates;

    for (int j = 1; j < numOfLables; j++)
    {
        int area = stats.at<int>(j, cv::CC_STAT_AREA);
        int left = stats.at<int>(j, cv::CC_STAT_LEFT);
        int top  = stats.at<int>(j, cv::CC_STAT_TOP);
        int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
        int height  = stats.at<int>(j, cv::CC_STAT_HEIGHT);

        bool bSizeIsTooSmall    = (area < 300);
        bool bWidthIsTooLong    = (width > height * 1.5);

        if(!bSizeIsTooSmall && !bWidthIsTooLong)
        {
            cv::Rect pogbox(left,top + roiPaulPogba.y,width,height);

            cv::rectangle(src, pogbox,cv::Scalar(255,255,0),2);

            if(pogboxCandidates.size() == 0 )
            {
                pogboxCandidates.push_back(pogbox);
            }
            else
            {
                int  insertIdx = -1;

                for(int i = pogboxCandidates.size() - 1 ; i >= 0; i--)
                {
                    if(pogbox.x <= pogboxCandidates.at(i).x)
                    {
                        insertIdx = i;
                    }
                }

                if(insertIdx != -1)
                {
                    vector<cv::Rect>::iterator it = pogboxCandidates.begin() + insertIdx;
                    pogboxCandidates.insert(it, pogbox);
                }
                else
                {
                    pogboxCandidates.push_back(pogbox);
                }
            }

            cv::rectangle(srcDrawing, pogbox,cv::Scalar(255,0,0),1);
        }

        std::cout<<area<<std::endl;
        /*
        if( area > 250 &&
            (double)area/width >= 0.5 &&
            !(width > height * 1.2))
        {
            cv::rectangle(src, cv::Rect(left,top,width,height),cv::Scalar(255,0,0),1);
            cv::Point midPoint = cv::Point(0,0);

            for(int y = top; y <= top + height; y++)
                for(int x = left; x <= left + width; x++)
                {
                    if(img_labels.at<int>(y, x) == j)
                    {
                        midPoint.x += x;
                        midPoint.y += y;
                    }
                }

            midPoint.x /= area;
            midPoint.y /= area;

            if(maxArea_2 < area)
            {
                maxArea_2 = area;
                midPoints[1] = midPoint;
                if(maxArea_2 > maxArea_1)
                {
                    cv::Point teampMidPoints = midPoints[1];
                    midPoints[1] = midPoints[0];
                    midPoints[0] = teampMidPoints;

                    int tempArea = maxArea_2;
                    maxArea_2 = maxArea_1;
                    maxArea_1 = tempArea;
                }
            }
        }
    }

    if(numOfLables > 3)
    {
        if(midPoints[0].x < midPoints[1].x)
        {
            cv::Point temp_point = midPoints[0];
            midPoints[0] = midPoints[1];
            midPoints[1] = temp_point;
        }

        double temp_degree=-(atan2(midPoints[0].y - midPoints[1].y, midPoints[0].x - midPoints[1].x)*180.0)/CV_PI;
        if(temp_degree<0.0)temp_degree+=180.0;

        std::cout<< temp_degree <<std::endl;
        if(temp_degree < 15.0  && midPoints[0] != cv::Point(0,0) && midPoints[1] != cv::Point(0,0)|| temp_degree >= 165.0 && midPoints[0] != cv::Point(0,0) && midPoints[1] != cv::Point(0,0))
        {
            //degree add
            cv::line(src, cv::Point(midPoints[0].x, midPoints[0].y+40), cv::Point(midPoints[1].x, midPoints[1].y+40), cv::Scalar(255,255,0), 2);
        }*/
    }

    static int nPogbaDetected = 0;
    bool pogbaDeteted = false;

    if(pogboxCandidates.size() >= 3)
    {
        int midIdx = pogboxCandidates.size()/2;

        std::cout <<"pogboxCandidates[midIdx].width: "<<pogboxCandidates[midIdx].width<<std::endl;
        int pogboxOptimalDistance = max(25, (int)((pogboxCandidates[midIdx].width)));


        int distanceSum     = 0;
        int distMidtoLeft   = pogboxCandidates[midIdx].x - (pogboxCandidates[midIdx -1].x + pogboxCandidates[midIdx -1].width);
        int distMidtoRight  = pogboxCandidates[midIdx + 1].x - (pogboxCandidates[midIdx].x + pogboxCandidates[midIdx].width);

        if( abs(distMidtoLeft - distMidtoRight) < 15)
        {
            int distanceSumThreshold = ( (distMidtoLeft + distMidtoRight)/2 ) * (pogboxCandidates.size() - 1);

            std::cout <<"-------------"<< std::endl;
            std::cout <<"distanceSumThreshold: " << distanceSumThreshold<< std::endl;

            for(int i = 1; i < pogboxCandidates.size(); i++)
            {
                std::cout <<"(pogboxCandidates[i].x - pogboxCandidates[i - 1].x = " << (pogboxCandidates[i].x - (pogboxCandidates[i - 1].x + pogboxCandidates[i - 1].width)) << std::endl;

                distanceSum += abs( pogboxOptimalDistance -  (pogboxCandidates[i].x - (pogboxCandidates[i - 1].x + pogboxCandidates[i - 1].width)));
            }

            if(distanceSum <= distanceSumThreshold)
            {
                nPogbaDetected++;
                pogbaDeteted = true;

                if(nPogbaDetected >= 3)
                {
                    for(int i = 0; i < pogboxCandidates.size(); i++)
                    {
                        cv::rectangle(srcDrawing, pogboxCandidates[i],cv::Scalar(0,255,0),2);
                    }

                    vision_msg.signal = POGBA;
                    std::cout <<"pogba"<< std::endl;
                }
            }
            else
            {
                std::cout <<"distanceSum constraint!!" << std::endl;
            }

            std::cout <<"distanceSum: " << distanceSum <<std::endl;
            std::cout <<"-------------"<< std::endl;
        }
    }

    if(!pogbaDeteted)
    {
        nPogbaDetected = 0;
    }
    /* static cv::Rect roiPaulPogba(0, 40, 320, 60);

    cv::Mat framePogba = src(roiPaulPogba).clone();
    cv::Mat frameHsv;

    cv::GaussianBlur(framePogba, framePogba, cv::Size(9,9), 1.5);
    cv::cvtColor(framePogba, frameHsv, cv::COLOR_BGR2HSV);
    cv::Mat framePogbaRedL, framePogbaRedH, framePogbaRedSum;

    cv::inRange(frameHsv, cv::Scalar(0, 35, 0), cv::Scalar(15, 255, 255), framePogbaRedL);
    cv::inRange(frameHsv, cv::Scalar(pogba_hue_min, 35, 0), cv::Scalar(pogba_hue_max, 255, 255), framePogbaRedH);

    cv::add(framePogbaRedL, framePogbaRedH, framePogbaRedSum);

    cv::Mat morphElemErode(17, 17, CV_8U, cv::Scalar(1));
    cv::Mat morphElemDilate(5, 5, CV_8U, cv::Scalar(1));

    cv::morphologyEx(framePogbaRedSum, framePogbaRedSum, cv::MORPH_ERODE, morphElemErode);
    cv::morphologyEx(framePogbaRedSum, framePogbaRedSum, cv::MORPH_DILATE, morphElemDilate);

    cv::Mat img_labels,stats, centroids;
    int numOfLables = cv::connectedComponentsWithStats(framePogbaRedSum, img_labels, stats, centroids, 4, CV_32S);

    cv::Point midPoints[2];
    int maxArea_2 = 0;
    int maxArea_1 = 0;

    for (int j = 1; j < numOfLables; j++)
    {
        int area    = stats.at<int>(j, cv::CC_STAT_AREA);
        int left    = stats.at<int>(j, cv::CC_STAT_LEFT);
        int top     = stats.at<int>(j, cv::CC_STAT_TOP);
        int width   = stats.at<int>(j, cv::CC_STAT_WIDTH);
        int height  = stats.at<int>(j, cv::CC_STAT_HEIGHT);

        if(area > 250 && (double)area/width >= 0.5)
        {
            cv::Point midPoint = cv::Point(0,0);

            for(int y = top; y <= top + height; y++)
                for(int x = left; x <= left + width; x++)
                {
                    if(img_labels.at<int>(y, x) == j)
                    {
                        midPoint.x += x;
                        midPoint.y += y;
                    }
                }

            midPoint.x /= area;
            midPoint.y /= area;

            if(maxArea_2 < area)
            {
                maxArea_2 = area;
                midPoints[1] = midPoint;
                if(maxArea_2 > maxArea_1)
                {
                    cv::Point teampMidPoints = midPoints[1];
                    midPoints[1] = midPoints[0];
                    midPoints[0] = teampMidPoints;

                    int tempArea = maxArea_2;
                    maxArea_2 = maxArea_1;
                    maxArea_1 = tempArea;
                }
            }
        }
    }

    if(numOfLables > 3)
    {
        if(midPoints[0].x < midPoints[1].x)
        {
            cv::Point temp_point = midPoints[0];
            midPoints[0] = midPoints[1];
            midPoints[1] = temp_point;
        }

        double temp_degree=-(atan2(midPoints[0].y - midPoints[1].y, midPoints[0].x - midPoints[1].x)*180.0)/CV_PI;
        if(temp_degree<0.0)temp_degree+=180.0;

        if(temp_degree < 30.0  && midPoints[0] != cv::Point(0,0) && midPoints[1] != cv::Point(0,0)|| temp_degree > 140.0 && midPoints[0] != cv::Point(0,0) && midPoints[1] != cv::Point(0,0))
        {
            vision_msg.signal = POGBA;
            //degree add
            cv::line(src, cv::Point(midPoints[0].x, midPoints[0].y+roiPaulPogba.y), cv::Point(midPoints[1].x, midPoints[1].y+roiPaulPogba.y), cv::Scalar(255,255,0), 2);
        }
    }*/
}

void LifeVision::_parkingProcess(cv::Mat &src)
{

    static bool bParkingFinished = false;

    if( siftParking.process( srcDrawing, cv::Rect(0 , 0, src.cols, src.rows * 0.5), true) && bParkingFinished == false)
    {
        parking_mark_cnt ++;
        if(parking_mark_cnt > 7)
        {
            parking_mode = true;
            vision_msg.parking_mode = true;
            bParkingFinished = true;
        }
    }
    else
    {
        vision_msg.parking_mode = false;
        parking_mark_cnt = 0;

        if(victory_data == 777)
        {
            parking_mode = false;
            victory_data = 0;
        }
    }
}

void LifeVision::_acrossProcess(Mat &src)
{
    cv::Rect roi(0, 0, src.cols, src.rows * 0.5);
    cv::Mat frameAcross = src(roi).clone();
    cv::cvtColor(frameAcross, frameAcross, CV_BGR2RGB);

    float *detections = (float*)calloc(255*6, sizeof(float));
    int num_output_class = 0;
    double threshold = .5;
    double hier_thresh = .0;
    vector<cv::Rect> results;
    vector<int> labelResults;
    cv::Rect acrossRoi;

    detect_mat(frameAcross,
               detections,
               &num_output_class,
               threshold,
               hier_thresh,
               results, labelResults);
    free(detections);


    int curAcrossSignal = NOT_DETECTED;
    static int pastAcrossSignal = 0;

    for(int i = 0; i < results.size(); i++)
    {
        results[i].x += roi.x;
        results[i].y += roi.y;

        std::cout << "across area : "<< results[i].size().area() << std::endl;
        std::cout << "across y : "<< results[i].y  << std::endl;
        std::cout << "across mid y : "<< results[i].y + results[i].height/2  << std::endl;

        acrossRoi = results[i];

        if(labelResults[i] == 0) //left
        {
            cv::rectangle(srcDrawing, results[i], cv::Scalar(0,255,0));
            curAcrossSignal = ACROSS_LEFT;
        }
        else
        {
            cv::rectangle(srcDrawing, results[i], cv::Scalar(0,0,255));
            curAcrossSignal = ACROSS_RIGHT;
        }
    }

    static bool isTracking = false;
    static bool bTackingIsFinished = false;

    static int miss_cnt = 4;
    if(results.size() != 0 && !across_flag) //Detected
    {
        across_mark_cnt++;

        if(across_mark_cnt > 2)
        {
            isTracking= true;
        }
        if(isTracking)
        {
            miss_cnt = 0;

            int targetBias = 0;
            int areaTh = 3300;

            if(curAcrossSignal == ACROSS_RIGHT)
            {
                targetBias = 15;
                areaTh = 2900;
            }

            vision_msg.tracking_point = acrossRoi.x + acrossRoi.width/2 - targetBias;

            cv::circle(src, cv::Point(acrossRoi.x + acrossRoi.width/2, acrossRoi.y + acrossRoi.height/2), 5, cv::Scalar(0,255,0),3);
            std::cout << "acrossRoi.area() = " << acrossRoi.area() << std::endl;

            if(abs(vision_msg.tracking_point - 162 - targetBias) < 20)//acrossRoi.area() >= 3200)
            {
                bTackingIsFinished = true;
            }

            std::cout << "(vision_msg.tracking_point - 162) = " << abs(vision_msg.tracking_point - 162) << std::endl;

            if( acrossRoi.area() >= areaTh &&
                    bTackingIsFinished == true &&
                    curAcrossSignal == pastAcrossSignal)
            {
                std::cout << "across signal tracking success" << std::endl;

                std::cout << "across signal checked" << std::endl;
                vision_msg.tracking_point = 0;
                vision_msg.signal = curAcrossSignal;
                across_flag = true;
                across_mark_cnt = 0;
            }
        }

        pastAcrossSignal = curAcrossSignal;
    }
    else
    {
        miss_cnt++;

        if(miss_cnt > 3)
        {
            across_mark_cnt = 0;
            vision_msg.tracking_point = 0;
        }
    }
}

void LifeVision::_drawHelpInform(Mat &src)
{
    cv::circle(src, cv::Point(POINT_0_X, POINT_0_Y+120), 3, cv::Scalar(0,255,0),2);
    cv::circle(src, cv::Point(POINT_1_X, POINT_1_Y+120), 3, cv::Scalar(0,255,0),2);
    cv::circle(src, cv::Point(POINT_2_X, POINT_2_Y+120), 3, cv::Scalar(0,255,0),2);
    cv::circle(src, cv::Point(POINT_3_X, POINT_3_Y+120), 3, cv::Scalar(0,255,0),2);

    //cv::Rect trafficLightROI(250, 95, 25, 20);
   // cv::rectangle(src, trafficLightROI, cv::Scalar(0,255,0),2);
}

void LifeVision::__lineJudge(Mat &src, Mat &edge, Mat &leftLine, Mat &rightLine, bool parking_mode)
{
    cv::Mat srcHsv, srcGray;

    cv::cvtColor(src, srcHsv,CV_BGR2HSV);
    cv::cvtColor(src, srcGray,CV_BGR2GRAY);

    leftLine  = cv::Mat::zeros(edge.size(), CV_8UC1);
    rightLine = cv::Mat::zeros(edge.size(), CV_8UC1);

    cv::Mat frameMaskYellow, frameMaskWhite;

    cv::inRange(srcHsv, cv::Scalar(20, 35, 35), cv::Scalar(45, 255, 255), frameMaskYellow);
    cv::inRange(srcGray, cv::Scalar(100), cv::Scalar(255), frameMaskWhite);

    cv::Mat morphElem(3, 3, CV_8U, cv::Scalar(1));
    cv::dilate(frameMaskYellow, frameMaskYellow, morphElem,cv::Point());
    cv::dilate(frameMaskWhite, frameMaskWhite, morphElem,cv::Point());

    int kernelSize = 1;
    for(int i = kernelSize; i < edge.rows - kernelSize; i++)
    {
        for(int j = kernelSize; j < edge.cols - kernelSize; j++)
        {
            if(edge.at<uchar>(i, j) != 0)
            {
                int nYellow = 0;
                int nWhite  = 0;

                for(int dy = -kernelSize; dy <= kernelSize; dy++)
                    for(int dx = -kernelSize; dx <= kernelSize; dx++)
                    {
                        if(frameMaskYellow.at<uchar>(i + dy, j + dx) != 0)
                            nYellow++;
                        if(frameMaskWhite.at<uchar>(i + dy, j + dx) != 0)
                            nWhite++;
                    }

                if(!parking_mode)
                {
                    if(nYellow >= 3)
                        leftLine.at<uchar>(i, j) = 255;
                    else
                    {
                        if(nWhite >= 3)
                        rightLine.at<uchar>(i, j) = 255;
                    }
                }
                else
                {
                    if(j < edge.cols/2)
                        leftLine.at<uchar>(i, j) = 255;
                    else
                        rightLine.at<uchar>(i, j) = 255;
                }
            }
        }
    }
}
