/**
 * @file /include/turtle_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

#ifndef turtle_master_QNODE_HPP_
#define turtle_master_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <QString>

#include "life_driving.hpp"
#include "../../../devel/include/turtle_vision/vision_msg.h"
#endif

namespace turtle_master {

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();

    LifeDriving life_driving;
    bool bStartAtuoDriving;
    bool is_goal_in;
    double my_imu;

    QString missionState;
Q_SIGNALS:
    void rosShutdown();
    void turtlebot3_state_update();
    void turtlebot3_velocity_update();

private:
    int init_argc;
    char** init_argv;

    double imu_w;
    double imu_x;
    double imu_y;
    double imu_z;

    bool imu_init_flag;

    double init_pos_x;
    double init_pos_y;

    double goal_pos_x;
    double goal_pos_y;


    enum {POGBA = 1, GREEN_LIGHT, PARKING, ACROSS_LEFT, ACROSS_RIGHT};
    enum {GO_LEFT, GO_STRAIGHT, DO_PARKING, ESCAPE, PARKING_COMPLETE};
    enum {TUNNEL_GO, TUNNEL_TURN, TUNNEL_ESCAPE};
    bool lineMsgUpdateFlag;

    // tunnel data
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::PolygonStamped foot_print;
    geometry_msgs::PolygonStamped before_pos;

    int tunnelEscapeState;
    double escape_imu;
    double diff_foot_print[4][2];
    double past_foot_print[4][2];
    // victory data
    std_msgs::Int32 victory;

    // publisher
    ros::Publisher cmd_vel_pub;
    ros::Publisher initial_pose_pub;
    ros::Publisher goal_pose_pub;
    ros::Publisher victory_pub;

    // subscriber
    ros::Subscriber vision_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber psd_sub;
    ros::Subscriber cds_sub;
    ros::Subscriber button_sub;
    ros::Subscriber goal_status_sub;
    ros::Subscriber goal_arrival_status_sub;
    ros::Subscriber laser_sub;

    void visionMsgCallback(const turtle_vision::vision_msg::ConstPtr &vision_msg);
    void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu);
    void psdMsgCallback(const std_msgs::Int32 &psd_val);
    void cdsMsgCallback(const std_msgs::Int32 &cds_val);
    void buttonMsgCallback(const std_msgs::Int32 &buttons_val);
//    void goalMsgCallback(const geometry_msgs::PolygonStamped &goal_status);
    void goalStateCallback(const move_base_msgs::MoveBaseActionResult &goal_arrival_status);
    void laserMsgCallback(const sensor_msgs::LaserScan &laser_arr);
    void footPrintMsgCallback(const geometry_msgs::PolygonStamped &foot_print);

    void stop();
    void tunnelInit();
    void escapeTunnel();
    bool turnToOtherSide();
    void escapeAcrossMission();
    void rvizInit();

};

}  // namespace turtle_master

#endif /* turtle_master_QNODE_HPP_ */
