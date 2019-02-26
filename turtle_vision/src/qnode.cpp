/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/turtle_vision/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace turtle_vision {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    button_data(0),
    nPicNum(0)
    {
       // output_video = string("robotis_good.avi");

//        outputVideo.open(output_video,
//            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
//            30,
//            cv::Size(320, 240),
//            true);
    }

QNode::~QNode() {

    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"turtle_vision");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  // msg_sub
  image_transport::ImageTransport it(n);
  image_sub = it.subscribe("/usb_cam/image_raw",1,&QNode::imageCallback,this);
  button_sub = n.subscribe("/button", 1, &QNode::buttonCallback, this);
  psd_sub = n.subscribe("/psd", 1, &QNode::psdMsgCallback, this);
  victory_sub = n.subscribe("/victory", 1, &QNode::victoryMsgCallback, this);

  // msg_pub
  vision_pub = n.advertise<turtle_vision::vision_msg>("vision_msg",1);
  lineEdge_pub = it.advertise("/life_cam/image_edge", 1);

  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(100);
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg_img)
{
    cv::Mat img_raw;

    try
    {
        img_raw = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

   // outputVideo << img_raw;
    life_vision.process(img_raw);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", life_vision.img_line_edge).toImageMsg();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", life_vision.img_line_edge).toImageMsg();

    lineEdge_pub.publish(msg);

    vision_pub.publish(life_vision.vision_msg);

    Q_EMIT updateImg(img_raw.clone());
}

void QNode::buttonCallback(const std_msgs::Int32 &button_data)
{
    this->button_data = button_data.data;
}

void QNode::psdMsgCallback(const std_msgs::Int32 &psd_val)
{
    life_vision.psd_val = psd_val.data;
}

void QNode::victoryMsgCallback(const std_msgs::Int32 &victory_data)
{
    life_vision.victory_data = victory_data.data;

    if(victory_data.data == 777)
    {
        std::cout<<"parking mode false" << std::endl;
        life_vision.parking_mode = false;
    }
}
}  // namespace turtle_vision
