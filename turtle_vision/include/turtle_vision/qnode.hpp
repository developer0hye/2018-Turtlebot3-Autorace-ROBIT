/**
 * @file /include/turtle_vision/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

#ifndef turtle_vision_QNODE_HPP_
#define turtle_vision_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>

#include "lifevision.hpp"

#endif

namespace turtle_vision {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();

    LifeVision life_vision;

Q_SIGNALS:
    void rosShutdown();
    void updateImg(cv::Mat img_raw);

private:
    int init_argc;
    char** init_argv;

    int button_data;
    // sub
    image_transport::Subscriber image_sub;
    ros::Subscriber button_sub;
    ros::Subscriber psd_sub;
    ros::Subscriber victory_sub;

    // pub
    ros::Publisher vision_pub;
    image_transport::Publisher lineEdge_pub;

    void imageCallback(const sensor_msgs::ImageConstPtr& img_cam);
    void buttonCallback(const std_msgs::Int32& button_data);
    void psdMsgCallback(const std_msgs::Int32 &psd_val);
    void victoryMsgCallback(const std_msgs::Int32 &victory_data);
    cv::VideoWriter outputVideo;
    std::string output_video;


    int nPicNum;

};

}  // namespace turtle_vision

#endif /* turtle_vision_QNODE_HPP_ */
