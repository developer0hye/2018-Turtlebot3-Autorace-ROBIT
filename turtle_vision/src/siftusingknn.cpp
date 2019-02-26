#include "../include/turtle_vision/siftusingknn.hpp"

using namespace std;
using namespace cv;

bool SiftUsingKNN::process(Mat& scene, Rect roi, bool viewMatches)
{
    if( img_object.empty() || scene.empty() ) {
        std::cout<< "Error reading images." << std::endl;
        return false;
    }

    objPoint = cv::Point(0,0);

    vector<KeyPoint> keypoints_scene; // keypoints
    Mat descriptors_scene; // descriptors (features)

    sift->detectAndCompute( scene(roi), noArray(), keypoints_scene, descriptors_scene );

    BFMatcher matcher;
    std::vector<std::vector<cv::DMatch> > matches;
    matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);

    std::vector<cv::DMatch> good_matches;
    const double nndrRatio = 0.8;
    for(size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].size() < 2)
            continue;

        const DMatch &m1 = matches[i][0];
        const DMatch &m2 = matches[i][1];

        if(m1.distance <= nndrRatio * m2.distance)
            good_matches.push_back(m1);
    }

    keypointsNum = good_matches.size();

    //-- Step 5: Draw lines between the good matching points
    if(good_matches.size() > matchingPointThreshold)
    {
        if(viewMatches == true)
        {
            Mat img_matches;
            drawMatches( img_object, keypoints_object, scene(roi), keypoints_scene,
                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                         vector<char>(), DrawMatchesFlags::DEFAULT );

            //-- Step 6: Localize the object inside the scene image with a square
            localizeInImage( good_matches, keypoints_object, keypoints_scene, img_object, img_matches, scene, (roi) );

            return true;
        }
    }
    return false;
}

void SiftUsingKNN::localizeInImage(const std::vector<DMatch>& good_matches,
                                   const std::vector<KeyPoint>& keypoints_object,
                                   const std::vector<KeyPoint>& keypoints_scene, const Mat& img_object,
                                   const Mat& img_matches, Mat& img_scene, cv::Rect roi)
{
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    cv::Point objCenter(0,0);
    for (int i = 0; i < good_matches.size(); i++){
        //-- Get the keypoints from the good matches
        cv::circle(img_scene, keypoints_scene[good_matches[i].trainIdx].pt, 3 , cv::Scalar(0,0,255), -1);
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    try {
        Mat H = findHomography(obj, scene, RANSAC);
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols-1, 0);
        obj_corners[2] = cvPoint(img_object.cols-1, img_object.rows-1);
        obj_corners[3] = cvPoint(0, img_object.rows-1);
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform(obj_corners, scene_corners, H);

        //draw scene

        for(int i = 0; i <4; i++)
        {
            scene_corners[i].x += roi.x;
            scene_corners[i].y += roi.y;
        }

        double l[4];
        double avgLength = 0.0;

        for(int i = 0; i < scene_corners.size(); i++)
        {
            l[i] = cv::norm(scene_corners[i] - scene_corners[ (i + 1) % scene_corners.size()]);
            avgLength += l[i];
        }
        avgLength /= 4.0;

        double diff = 0.0;
        for(int i = 0; i < 4; i++)
        {
            diff += fabs(avgLength - l[i]);
        }


        if(diff > 50.0)
        {
            color = cv::Scalar(0,0,255);
        }
        else
        {
             color = cv::Scalar(0,255,0);
        }


        line(img_scene, scene_corners[0],
                scene_corners[1],
                color, 4);
        line(img_scene, scene_corners[1],
                scene_corners[2],
                color, 4);
        line(img_scene, scene_corners[2],
                scene_corners[3],
                color, 4);
        line(img_scene, scene_corners[3],
                scene_corners[0],
                color, 4);

        for(cv::Point point: scene_corners)
        {
            objCenter += point;
        }
        objCenter.x /= scene_corners.size();
        objCenter.y /= scene_corners.size();

        cv::circle(img_scene, objCenter, 3, color, -1);

        objPoint = objCenter;


    } catch (Exception& e) {}
}

