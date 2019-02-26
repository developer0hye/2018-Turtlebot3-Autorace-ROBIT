/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/turtle_master/qnode.hpp"


#define CHECK_IMU 0
#define CHECK_LIDAR 0
#define CHECK_PSD 0

#define PARK_OBSTACLE_PSD_VALUE 400

#define SAFETY_MODE 0

#define DEG2RAD(rad) ((rad)*180/M_PI)
#define TRAFFIC_LIGHT_GAIN 0.005
#define TRAFFIC_LIGHT_TARGET 53
#define TURN_GAIN 0.025

using namespace std;
namespace turtle_master {

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    imu_w(0.0),
    imu_x(0.0),
    imu_y(0.0),
    imu_z(0.0),
    bStartAtuoDriving(false),
    init_pos_x(0.63),
    init_pos_y(1.50),
    goal_pos_x(1.60),
    goal_pos_y(-1.45),
    lineMsgUpdateFlag(true),
    tunnelEscapeState(TUNNEL_GO),
    escape_imu(0.0),
    missionState("GO")
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"turtle_master");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // msg_pub
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    goal_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    victory_pub = n.advertise<std_msgs::Int32>("/victory",1);

    // msg_sub
    vision_sub = n.subscribe("/vision_msg", 1, &QNode::visionMsgCallback, this);
    imu_sub = n.subscribe("/imu", 1, &QNode::imuMsgCallback, this);
    psd_sub = n.subscribe("/psd", 1, &QNode::psdMsgCallback, this);
    cds_sub = n.subscribe("/cds", 1, &QNode::cdsMsgCallback, this);
    button_sub = n.subscribe("/button", 1, &QNode::buttonMsgCallback, this);
    goal_status_sub = n.subscribe("/move_base/global_costmap/footprint", 1, &QNode::footPrintMsgCallback, this);
    goal_arrival_status_sub = n.subscribe("/move_base/result", 1, &QNode::goalStateCallback, this);
    laser_sub = n.subscribe("scan", 1, &QNode::laserMsgCallback, this);

    // connect
    QObject::connect(this, SIGNAL(turtlebot3_velocity_update()), &life_driving, SLOT(turtlebot3_velocity_update()));
    start();

    return true;
}

void QNode::run() {
    ros::Rate loop_rate(1000);


    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::visionMsgCallback(const turtle_vision::vision_msg::ConstPtr& vision_msg)
{

    Q_EMIT turtlebot3_state_update();
    switch(life_driving.drive_state)
    {
    case LifeDriving::TURTLE_STOP:
        missionState = "TURTLE_STOP";
        std::cout<<"Waiting... Start Button Signal..."<<std::endl;
        stop();
        return;
        break;

    case LifeDriving::TURTLE_STOP_POGBA:
        static int pogbaCnt = 0;
        if(vision_msg->signal != POGBA)
        {
            lineMsgUpdateFlag        = true;
            life_driving.drive_state = LifeDriving::TURTLE_GO;
            pogbaCnt = 0;
        }
        else
        {
            static bool bTunnelInit = false;
            pogbaCnt ++;
            if(pogbaCnt > 2 && bTunnelInit == false)
            {
                rvizInit();
                bTunnelInit = true;
            }

            stop();
            return;
        }
        break;

    case LifeDriving::TURTLE_TRAFFIC_LIGHT_CHECK:
        std::cout<<"Traffic Light checking..."<<std::endl;

        missionState = "TURTLE_TRAFFIC_LIGHT_CHECK";
        if(vision_msg->signal == GREEN_LIGHT || true /*check*/)
        {
            std::cout<<"Green Light Detected! GO!"<<std::endl;
            life_driving.drive_state = LifeDriving::TURTLE_GO;
            return;
        }
        else
        {
            stop();
            return;
        }
        break;

    case LifeDriving::TURTLE_ACROSS:
    {
        missionState = "TURTLE_ACROSS";
        enum    ACROSS_STATE{ACROSS_TURN_TO_OTHER_SIDE, ACROSS_LINE_TRACKING, ACROSS_END};
        static  ACROSS_STATE state = ACROSS_TURN_TO_OTHER_SIDE;

        switch(state)
        {
        case ACROSS_TURN_TO_OTHER_SIDE:
        {
            missionState = "ACROSS_TURN_TO_OTHER_SIDE";
            lineMsgUpdateFlag = false;
            bool bTurnFinished = !turnToOtherSide();
            if(bTurnFinished)
            {
                if(life_driving.acrossDirection == ACROSS_LEFT)
                {
                    life_driving.setVelocity(0.22, 1.5);
                }
                else
                {
                    life_driving.setVelocity(0.22, -1.5);
                }
                state               = ACROSS_LINE_TRACKING;
            }
        }
            break;
        case ACROSS_LINE_TRACKING:
        {
            missionState = "ACROSS_LINE_TRACKING";
            lineMsgUpdateFlag = true;

            static bool     escapeFlag  = false;
            static double   past_imu    = my_imu;

            if(!escapeFlag)
            {
                if((fabs(my_imu) - 180.0) < 10.0)
                {
                    if(((past_imu-180.0) * (my_imu-180.0)) < 0.0)
                    {
                        escapeFlag = true;
                    }
                }
                past_imu = my_imu;
            }
            else
            {
                static int nEscapse = 0;
                if( !life_driving.bLeftLineDetected && !life_driving.bRightLineDetected)
                {
                    nEscapse ++;
                    if(nEscapse > 3)
                    {
                        std::cout <<"across line not detected!"<< std::endl;
                        if(life_driving.acrossDirection == ACROSS_LEFT)
                        {
                            life_driving.setVelocity(0.22, 0.92);
                        }
                        else if(life_driving.acrossDirection == ACROSS_RIGHT)
                        {
                            life_driving.setVelocity(0.22, -0.92);
                        }

                        state       = ACROSS_END;
                        escapeFlag  = false;
                        nEscapse    = 0;
                    }
                }
                else
                    nEscapse = 0;
            }
        }
            break;

        case ACROSS_END:

            missionState = "ACROSS_END";
            //initialize()
            state                           = ACROSS_TURN_TO_OTHER_SIDE;
            lineMsgUpdateFlag               = true;
            life_driving.acrossDirection    = 0;
            life_driving.drive_state        = LifeDriving::TURTLE_GO;

            std::cout <<"across end"<<std::endl;

            break;
        }
    }
        break;

    case LifeDriving::TURTLE_AVOID_OBSTACLE:
        missionState = "TURTLE_AVOID_OBSTACLE";
        enum OBSTACLE_STATE{OBSTACLE_HALF_MOON_TURN, OBSTACLE_EXIT};
        static OBSTACLE_STATE state = OBSTACLE_HALF_MOON_TURN;

        switch(state)
        {
        case OBSTACLE_HALF_MOON_TURN:
            missionState = "OBSTACLE_HALF_MOON_TURN";
            lineMsgUpdateFlag = false;
            std::cout <<"half_moon_turn" << std::endl;
            if(my_imu < 170.0)
            {
                life_driving.setVelocity(0.0, (185.0 - my_imu)*TURN_GAIN*2.0);
            }
            else if(my_imu > 170.0 && my_imu < 180.0)
            {
                life_driving.setVelocity(0.0, (185.0 - my_imu)*TURN_GAIN);
                msleep(1);
            }
            else if(my_imu >= 180.0)
            {
                life_driving.setVelocity(0.22, -1.5);
                state = OBSTACLE_EXIT;
            }

            break;

        case OBSTACLE_EXIT:
            missionState = "OBSTACLE_EXIT";

            if(fabs(my_imu - 55.0) < 3.0)
            {
                std::cout << "EXIT"  << std::endl;

                lineMsgUpdateFlag           = true;

                state                       = OBSTACLE_HALF_MOON_TURN;

                life_driving.setVelocity(0.22, 0.0);
                life_driving.drive_state    = LifeDriving::TURTLE_GO;
                lineMsgUpdateFlag = true;
            }

            break;
        }

        break;

    case LifeDriving::TURTLE_PARK:
    {
        missionState = "TURTLE_PARK";
        enum    PARK_STATE{PARK_GO_LEFT, PARK_GO_STRAIGHT , PARK_DO_PARKING, PARK_ESCAPE};
        static  PARK_STATE state = PARK_GO_LEFT;

        switch(state)
        {
        case PARK_GO_LEFT:
        {
            missionState = "PARK_GO_LEFT";
            lineMsgUpdateFlag = true;

            std::cout << "GO_LEFT" << std::endl;
            static int  nLeftLineNotCnt = 0;
            static bool leftlineNotDetected = false;

            if(!life_driving.bLeftLineDetected)
            {
                nLeftLineNotCnt ++;
            }
            else
            {
                nLeftLineNotCnt = 0;
            }

#if SAFETY_MODE
            if(nLeftLineNotCnt >= 12)
            {
                leftlineNotDetected = true;
            }

            if(leftlineNotDetected)
            {
                static int  nLeftLineDetectCnt = 0;
                static bool leftlineDetected   = false;

                if( fabs(life_driving.left_line_degree - 90.) <= 10 &&
                        fabs(life_driving.right_line_degree - 90.) <= 10)
                {
                    nLeftLineDetectCnt++;
                }
                else
                {
                    nLeftLineDetectCnt = 0;
                }

                if(nLeftLineDetectCnt >= 2)
                {
                    leftlineDetected = true;
                }

                if(leftlineDetected)
                {
                    if(fabs(270.0 - my_imu) > 3.0)
                    {
                        life_driving.setVelocity(0.0, (280.0 - my_imu) * TURN_GAIN * 1.8);
                    }
                    else
                    {
                        leftlineDetected = false;
                        nLeftLineDetectCnt = 0;

                        nLeftLineNotCnt = 0;
                        leftlineNotDetected = false;
                        life_driving.setVelocity(0.22, 0.0);

                        state                           = PARK_GO_STRAIGHT;



                    }
                }
            }
#else
            if(nLeftLineNotCnt > 13)
            {
                leftlineNotDetected = true;
            }

            if(leftlineNotDetected)
            {
                life_driving.setVelocity(0.22, 1.6);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                if(my_imu > 240.0 && life_driving.bRightLineDetected)
                {
                    nLeftLineNotCnt                 = 0;
                    leftlineNotDetected             = false;
                    state                           = PARK_GO_STRAIGHT;
                }
            }
#endif

        }
            break;

        case PARK_GO_STRAIGHT:
        {
            missionState = "PARK_GO_STRAIGHT";
            lineMsgUpdateFlag = true;

            std::cout << "GO_STRAIGT" << std::endl;
            static int isParkingLineCnt = 0;

            std::cout << "life_driving.psd_val = " << life_driving.psd_val << std::endl;
            if(life_driving.psd_val >= PARK_OBSTACLE_PSD_VALUE && fabs(my_imu - 270.0) <= 30.0)
            {
                life_driving.parkingPosition = LifeDriving::LEFT;
            }
            if(life_driving.right_line_degree == -1 && life_driving.left_line_degree == -1 && fabs(my_imu - 270.0) <= 20.0)
            {
                std::cout<<"parking horizontal line detected" << std::endl;
                isParkingLineCnt ++;
            }
            else
                isParkingLineCnt = 0;

            if(vision_msg->left_line_degree == -1. && vision_msg->right_line_degree == -1. && fabs(my_imu - 270.0) <= 20.0)
            {
                state = PARK_DO_PARKING;
                life_driving.right_line_degree = 0.;
                life_driving.left_line_degree  = 0.;
                life_driving.setVelocity(0.0, 0.0);
                isParkingLineCnt = 0;
            }
        }
            break;

        case PARK_DO_PARKING:
        {
            missionState = "PARK_DO_PARKING";
            static bool bba_ggu_complete = false;
            lineMsgUpdateFlag = false;

            if(life_driving.parkingPosition == LifeDriving::LEFT)
            {
                std::cout << "LEFT_PARKING" << std::endl;
#if SAFETY_MODE
                if(!bba_ggu_complete)
                {
                    if(my_imu < 359.0)
                    {
                        life_driving.setVelocity(0.0, (360.0 - my_imu)*TURN_GAIN*2.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    }
                    else if(my_imu >= 350.0 && my_imu < 359.0)
                    {
                        life_driving.setVelocity(0.0, (364.0 - my_imu)*TURN_GAIN);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1);
                    }
                    else if(my_imu >= 359.0 || my_imu < 1.0)
                    {

                        life_driving.setVelocity(0.22, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1450);
                        life_driving.setVelocity(0.0, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(10);
                        life_driving.setVelocity(-0.22, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1450);
                        life_driving.setVelocity(0.0, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                        bba_ggu_complete = true;
                    }
                }
                else
                {
                    if(fabs(90.0 - ((int)(my_imu + 2.0)%360)) > 3.0)
                    {
                        life_driving.setVelocity(0.0, (fabs(90.0 - ((int)(my_imu + 2.0)%360)) * TURN_GAIN * 1.8));
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    }
                    else
                    {
                        std::cout << "go PARK_ESCAPE" << std::endl;
                        state = PARK_ESCAPE;
                        bba_ggu_complete = false;
                        lineMsgUpdateFlag = true;
                    }
                }
#else

                life_driving.setVelocity(-0.22, -2.84);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(1200);

                life_driving.setVelocity(0.-22, 0.0);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(800);

                life_driving.setVelocity(0.22, 0.0);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(800);

                life_driving.setVelocity(0.22, -2.84);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(1000);

                state = PARK_ESCAPE;
                bba_ggu_complete = false;
                lineMsgUpdateFlag = true;
#endif
            }
            else if(life_driving.parkingPosition == LifeDriving::RIGHT)
            {
                std::cout << "RIHGT_PARKING" << std::endl;
#if SAFETY_MODE
                if(!bba_ggu_complete)
                {
                    if(my_imu >= 180.0)
                    {
                        life_driving.setVelocity(0.0, (175 - my_imu)*TURN_GAIN*1.8);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    }
                    else if(my_imu <= 190.0 && my_imu >= 180.0)
                    {
                        life_driving.setVelocity(0.0, (175 - my_imu)*TURN_GAIN);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1);
                    }
                    else if(my_imu < 180.0)
                    {
                        life_driving.setVelocity(0.22, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1350);
                        life_driving.setVelocity(0.0, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(10);
                        life_driving.setVelocity(-0.22, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                        msleep(1350);
                        life_driving.setVelocity(0.0, 0.0);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                        bba_ggu_complete = true;
                    }
                }
                else
                {
                    if(fabs(90.0 - my_imu) > 3.0)
                    {
                        life_driving.setVelocity(0.0, (90.0 - (my_imu)) * TURN_GAIN * 1.8);
                        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    }
                    else
                    {
                        std::cout << "go PARK_ESCAPE" << std::endl;
                        state = PARK_ESCAPE;
                        bba_ggu_complete = false;
                        lineMsgUpdateFlag = true;
                    }
                }
#else

                life_driving.setVelocity(-0.22, 2.84);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(1200);

                life_driving.setVelocity(0.-22, 0.0);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(800);

                life_driving.setVelocity(0.22, 0.0);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(800);

                life_driving.setVelocity(0.22, 2.84);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                msleep(1000);

                state = PARK_ESCAPE;
                bba_ggu_complete = false;
                lineMsgUpdateFlag = true;
#endif
            }
            break;
        }

        case PARK_ESCAPE:
        {
            missionState = "PARK_ESCAPE";
            std::cout << "PARK_ESCAPE" << std::endl;
#if SAFETY_MODE
            static bool bEscapeHorizontalLineDetected = false;

            if( vision_msg->left_line_degree == -1. &&
                    vision_msg->right_line_degree == -1.)
            {
                std::cout<<"hor line detected!"<<std::endl;
                if(fabs(my_imu - 90.0) <= 15.0)
                {
                    std::cout <<"+ imu condition checked" << std::endl;
                    bEscapeHorizontalLineDetected = true;
                }
            }

            if(bEscapeHorizontalLineDetected)
            {
                if(my_imu < 165.0)
                    life_driving.setVelocity(0.0, (180.0 - my_imu) * TURN_GAIN * 2.0);
                else
                {
                    victory.data = 777;

                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);
                    victory_pub.publish(victory);

                    state = PARK_GO_LEFT;
                    life_driving.drive_state = LifeDriving::TURTLE_GO;
                    bEscapeHorizontalLineDetected = false;
                }
            }
#else
            static int isEscapeLineCnt = 0;

            if(!life_driving.bLeftLineDetected && !life_driving.bRightLineDetected)
                isEscapeLineCnt ++;
            else
                isEscapeLineCnt = 0;

            if(isEscapeLineCnt > 10)
            {
                std::cout <<"park mission end" << std::endl;
                life_driving.setVelocity(0.22, 1.5);
                cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                victory.data = 777;

                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);
                victory_pub.publish(victory);

                state = PARK_GO_LEFT;

                isEscapeLineCnt = 0;

                life_driving.drive_state = LifeDriving::TURTLE_GO;
            }
#endif


        }
            break;
        }
    }
        break;

    case LifeDriving::TURTLE_TUNNEL:
    {
        missionState = "TURTLE_TUNNEL";
        enum    TUNNEL_STATE{TUNNEL_INIT, TUNNEL_ESCAPE_TUNNEL};
        static  TUNNEL_STATE state = TUNNEL_INIT;
        lineMsgUpdateFlag = false;

        switch (state)
        {
        case TUNNEL_INIT:
            missionState = "TUNNEL_INIT";
//            if(fabs(my_imu - 295.0) > 3.0)
//            {
//                life_driving.setVelocity(0.0, (300.0 - my_imu)*TURN_GAIN*2.0);
//                cmd_vel_pub.publish(life_driving.cmd_vel_msg);
//            }
//            else

            {
                tunnelInit();
                state = TUNNEL_ESCAPE_TUNNEL;
            }
            return ;
            break;
        case TUNNEL_ESCAPE_TUNNEL:
            missionState = "TUNNEL_ESCAPE_TUNNEL";

            if(my_imu > 180.0)
                escape_imu = 360.0 - my_imu;
            else
                escape_imu = -my_imu;

            std::cout << "tunnel gogogogo" << std::endl;

            if(life_driving.arrive_goal && tunnelEscapeState == TUNNEL_ESCAPE)
            {
                life_driving.setLineInformation(*vision_msg);

                static int nLineDetectedCount = 0;
                static int nPsdLongDistanceCount = 0;

//                if(life_driving.psd_val < 600)
//                {
//                    nPsdLongDistanceCount++;
//                }
//                else
//                {
//                    nPsdLongDistanceCount = 0;
//                }

                if(life_driving.bRightLineDetected == true || life_driving.bLeftLineDetected == true)
                {
                    nLineDetectedCount++;
                }
                else
                {
                    nLineDetectedCount = 0;
                }

                if(nLineDetectedCount >= 10)
                {
//                    stop();

                    std::cout<<"Tunnel Escape" << std::endl;
                    state                       = TUNNEL_INIT;
                    life_driving.drive_state    = LifeDriving::TURTLE_GO;
                    lineMsgUpdateFlag = true;
                    nLineDetectedCount  = 0;
                }
                else
                {

                    life_driving.setVelocity(0.22, escape_imu * TURN_GAIN);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    std::cout << "tunnel_escape_yaw = " << escape_imu * TURN_GAIN << std::endl;
                }
            }
            else
            {
                escapeTunnel();
            }
            return;
            break;
        }
    }
        break;

    case LifeDriving::TURTLE_GO:

        missionState = "TURTLE_GO";
        //std::cout <<"turtle go"<<std::endl;
        life_driving.setTrackingPoint(*vision_msg);

        switch(vision_msg->signal)
        {
        case POGBA:
            if((my_imu >= 0 && my_imu < 30.0) || (my_imu <= 360.0 && my_imu > 330.0))
            {
                missionState = "POGBA";
                std::cout <<"pogba go"<<std::endl;
                lineMsgUpdateFlag        = false;
                life_driving.drive_state = LifeDriving::TURTLE_STOP_POGBA;
                stop();
                return;
            }
            break;
        case ACROSS_LEFT:
            missionState = "ACROSS_LEFT";
        case ACROSS_RIGHT:
            missionState = "ACROSS_RIGHT";
            std::cout <<"across go"<<std::endl;
            life_driving.acrossDirection    = vision_msg->signal;
            life_driving.drive_state        = LifeDriving::TURTLE_ACROSS;
            life_driving.setTrackingPoint(0);
            stop();
            return;
            break;
        }

        if(vision_msg->parking_mode == true)
        {
            std::cout <<"parking_mode go"<<std::endl;
            if(fabs(my_imu - 180.0) < 30.0)
                life_driving.drive_state  = LifeDriving::TURTLE_PARK;
            return ;
        }
        else if(life_driving.bObstacleDetected  == true)
        {
            std::cout <<"obstacle go"<<std::endl;
            life_driving.drive_state  = LifeDriving::TURTLE_AVOID_OBSTACLE;
            lineMsgUpdateFlag         = false;
            return ;
        }
        else if(life_driving.cds_val &&
                life_driving.psd_val > 600 &&
                fabs(my_imu - 270.) <= 30.)
        {
            std::cout<<"tunnel go" <<std::endl;
            life_driving.drive_state  = LifeDriving::TURTLE_TUNNEL;
            lineMsgUpdateFlag         = false;
            return ;
        }

        break;
    }


    if(lineMsgUpdateFlag == true)
    {
        life_driving.setLineInformation(*vision_msg);
        Q_EMIT      turtlebot3_velocity_update();
    }

    cmd_vel_pub.publish(life_driving.cmd_vel_msg);


    return ;



    if(life_driving.is_in_tunnel)
    {
        escapeTunnel();
        return;
    }

    // tunnel
    if(life_driving.cds_val && life_driving.psd_val > 600 && life_driving.parkingDrivingMode == ESCAPE)
    {
        tunnelInit();
    }

    life_driving.tracking_point = vision_msg->tracking_point;

    if(vision_msg->parking_mode == true)
        life_driving.parking_mode = true;

    if(vision_msg->signal == ACROSS_LEFT
            || vision_msg->signal == ACROSS_RIGHT
            || (life_driving.bObstacleDetected && !life_driving.isObstacleMissionStart))
    {
        stop();

        if( vision_msg->signal == ACROSS_LEFT   ||
                vision_msg->signal == ACROSS_RIGHT)
        {
            life_driving.acrossDirection = vision_msg->signal;
            life_driving.acrossState = LifeDriving::ACROSS_DEPARTURE;
        }
        else if(life_driving.bObstacleDetected  && !life_driving.isObstacleMissionStart)
        {
            life_driving.isObstacleMissionStart = true;
            life_driving.avoidMissionState = LifeDriving::TURN;
        }
    }
    else if(life_driving.acrossState == LifeDriving::ACROSS_DEPARTURE)
    {
        stop();
        bool bTurnFinished = !turnToOtherSide();
        lineMsgUpdateFlag = false;
        if(bTurnFinished)
        {
            if(life_driving.acrossDirection == ACROSS_LEFT)
                life_driving.setVelocity(0.22, 1.5);
            else
                life_driving.setVelocity(0.22, -1.5);
            life_driving.acrossState = LifeDriving::ACROSS_ARRIVAL;

            lineMsgUpdateFlag = true;
        }
    }
    else if(life_driving.acrossState == LifeDriving::ACROSS_ARRIVAL)
    {
        static bool escapeFlag = false;
        static double past_imu = my_imu;
        if(!escapeFlag)
        {
            if((fabs(my_imu) - 180.0) < 10.0)
            {
                if(((past_imu-180.0) * (my_imu-180.0)) < 0.0)
                {
                    escapeFlag = true;
                }
            }
            past_imu = my_imu;
        }
        else
        {
            static int nEscapse = 0;
            if( !life_driving.bLeftLineDetected && !life_driving.bRightLineDetected)
            {
                nEscapse ++;
                if(nEscapse > 3)
                {
                    if(life_driving.acrossDirection == ACROSS_LEFT)
                    {
                        life_driving.setVelocity(0.22, 1.3);
                    }
                    else if(life_driving.acrossDirection == ACROSS_RIGHT)
                    {
                        life_driving.setVelocity(0.22, -1.5);
                    }
                    life_driving.acrossDirection = 0;
                    life_driving.acrossState = LifeDriving::ACROSS_NORMAL;
                }
            }
            else
                nEscapse = 0;
        }
    }
    else if(life_driving.isObstacleMissionStart && life_driving.avoidMissionState == LifeDriving::TURN)
    {
        std::cout<<"obstacle turn"<<std::endl;
        lineMsgUpdateFlag = false;

        if(my_imu < 180.0)
        {
            life_driving.setVelocity(0.0, (185.0 - my_imu)*TURN_GAIN*2.0);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);
        }
        //        else if(my_imu > 170.0 && my_imu < 180.0)
        //        {
        //            life_driving.setVelocity(0.0, (185.0 - my_imu)*TURN_GAIN);
        //            cmd_vel_pub.publish(life_driving.cmd_vel_msg);
        //            msleep(1);
        //        }
        else if(my_imu >= 180.0)
        {
            life_driving.setVelocity(0.22, -1.5);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);
            life_driving.avoidMissionState = life_driving.GO;
        }
    }
    else if(life_driving.isObstacleMissionStart && life_driving.avoidMissionState == life_driving.GO)
    {
        if(fabs(my_imu - 55.0) < 3.0)
        {
            lineMsgUpdateFlag = true;
            life_driving.setVelocity(0.22, 0.0);
            life_driving.avoidMissionState = life_driving.OBSTACLE_COMPLETE;
            life_driving.isObstacleMissionStart = false;
        }
    }
    else if(life_driving.parking_mode && life_driving.parkingDrivingMode == GO_LEFT)
    {
        std::cout << "GO_LEFT" << std::endl;
        static int nLeftLineNotCnt = 0;
        static bool leftlineNotDetected = false;

        if(!life_driving.bLeftLineDetected)
            nLeftLineNotCnt ++;
        else
            nLeftLineNotCnt = 0;

        if(nLeftLineNotCnt > 15)
            leftlineNotDetected = true;

        if(leftlineNotDetected)
        {
            life_driving.setVelocity(0.22, 1.5);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);

            if(my_imu > 225.0 && life_driving.bRightLineDetected)
            {
                leftlineNotDetected = false;
                life_driving.parkingDrivingMode = GO_STRAIGHT;
            }
        }
    }
    else if(life_driving.parking_mode && life_driving.parkingDrivingMode == GO_STRAIGHT)
    {
        std::cout << "GO_STRAIGT" << std::endl;
        static int isParkingLineCnt = 0;

        std::cout << "life_driving.psd_val = " << life_driving.psd_val << std::endl;
        if(life_driving.psd_val > 500)
            life_driving.parkingPosition = LifeDriving::LEFT;

        if(!life_driving.bLeftLineDetected && !life_driving.bRightLineDetected)
            isParkingLineCnt ++;
        else
            isParkingLineCnt = 0;

        if(isParkingLineCnt > 3)
        {
            life_driving.parkingDrivingMode = DO_PARKING;
            life_driving.setVelocity(0.0, 0.0);
        }
    }
    else if(life_driving.parking_mode && life_driving.parkingDrivingMode == DO_PARKING)
    {
        static bool bba_ggu_complete = false;
        lineMsgUpdateFlag = false;
        if(life_driving.parkingPosition == LifeDriving::LEFT)
        {
            if(!bba_ggu_complete)
            {
                if(my_imu < 359.0)
                {
                    life_driving.setVelocity(0.0, (360.0 - my_imu)*TURN_GAIN*2.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                }
                //                else if(my_imu >= 350.0 && my_imu < 359.0)
                //                {
                //                    life_driving.setVelocity(0.0, (364.0 - my_imu)*TURN_GAIN);
                //                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                //                    msleep(1);
                //                }
                else if(my_imu >= 359.0 || my_imu < 1.0)
                {
                    life_driving.setVelocity(0.22, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1350);
                    life_driving.setVelocity(0.0, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1000);
                    life_driving.setVelocity(-0.22, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1350);
                    life_driving.setVelocity(0.0, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                    bba_ggu_complete = true;
                }
            }
            else
            {
                if(fabs(90.0 - ((int)(my_imu + 2.0)%360)) > 3.0)
                {
                    life_driving.setVelocity(0.0, (90.0 - ((int)(my_imu + 2.0)%360)) * TURN_GAIN * 1.8);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                }
                else
                {
                    std::cout << "escape" << std::endl;
                    life_driving.parkingDrivingMode = ESCAPE;
                }
            }
        }
        else if(life_driving.parkingPosition == life_driving.RIGHT)
        {
            if(!bba_ggu_complete)
            {
                if(my_imu >= 180.0)
                {
                    life_driving.setVelocity(0.0, (175 - my_imu)*TURN_GAIN*2.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                }
                //                else if(my_imu <= 190.0 && my_imu >= 180.0)
                //                {
                //                    life_driving.setVelocity(0.0, (175 - my_imu)*TURN_GAIN);
                //                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                //                    msleep(1);
                //                }
                else if(my_imu < 180.0)
                {
                    life_driving.setVelocity(0.22, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1350);
                    life_driving.setVelocity(0.0, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1000);
                    life_driving.setVelocity(-0.22, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                    msleep(1350);
                    life_driving.setVelocity(0.0, 0.0);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);

                    bba_ggu_complete = true;
                }
            }
            else
            {
                if(fabs(90.0 - my_imu) > 3.0)
                {
                    life_driving.setVelocity(0.0, (90.0 - (my_imu)) * TURN_GAIN * 1.8);
                    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
                }
                else
                    life_driving.parkingDrivingMode = ESCAPE;
            }
        }
    }
    else if(life_driving.parking_mode && life_driving.parkingDrivingMode == ESCAPE)
    {
        std::cout << "ESCAPE" << std::endl;

        lineMsgUpdateFlag = true;

        static int isEscapeLineCnt = 0;
        if(!life_driving.bLeftLineDetected && !life_driving.bRightLineDetected)
            isEscapeLineCnt ++;
        else
            isEscapeLineCnt = 0;

        if(isEscapeLineCnt > 10)
        {
            life_driving.setVelocity(0.22, 1.5);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);

            life_driving.parking_mode = false;
            victory.data = 777;
            victory_pub.publish(victory);

            life_driving.parkingDrivingMode = PARKING_COMPLETE;
        }
    }
    else
    {
        life_driving.setLineInformation(*vision_msg);
    }

    if(lineMsgUpdateFlag)
    {
        life_driving.setLineInformation(*vision_msg);
        Q_EMIT turtlebot3_velocity_update();
    }

    Q_EMIT turtlebot3_state_update();
    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
}

void QNode::imuMsgCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    imu_w = imu->orientation.w;
    imu_z = imu->orientation.z;
    imu_x = imu->orientation.x;
    imu_y = imu->orientation.y;

    double yaw = atan2(2*imu_x*imu_y + 2*imu_w*imu_z, imu_w*imu_w + imu_x*imu_x - imu_y*imu_y - imu_z*imu_z);

    // get yaw angle of the turtlebot
    life_driving.imu_yaw = DEG2RAD(yaw);
    my_imu = life_driving.imu_yaw;
    if(my_imu < 0)
        my_imu += 360.0;

    int tempQuotient = 0;
    tempQuotient = life_driving.imu_yaw/90;
    life_driving.imu_yaw -= tempQuotient*90.0;


#if CHECK_IMU
    cout <<"my_imu: " << my_imu <<endl;
#endif

}

void QNode::psdMsgCallback(const std_msgs::Int32 &psd_val)
{
    life_driving.psd_val = psd_val.data;

#if CHECK_PSD
    std::cout<<"psd_val.data: " << psd_val.data <<std::endl;
#endif
}

void QNode::cdsMsgCallback(const std_msgs::Int32 &cds_val)
{
    life_driving.cds_val = cds_val.data;
}

void QNode::buttonMsgCallback(const std_msgs::Int32 &button_val)
{
    static bool bClicked = false;

    if(button_val.data == 1 && bClicked == false)
    {
        std::cout<<"GOGOGO..."<<std::endl;
//        if(bClicked == false)
//            rvizInit();

        bClicked = true;
        //bStartAtuoDriving = true;
        if(life_driving.drive_state != LifeDriving::TURTLE_STOP)
        {
            life_driving.past_drvie_state = life_driving.drive_state;
            life_driving.drive_state = LifeDriving::TURTLE_STOP;
        }
        else //cur drive state == STOP
        {
            if(life_driving.past_drvie_state == LifeDriving::TURTLE_STOP)
            {
                life_driving.drive_state = LifeDriving::TURTLE_TRAFFIC_LIGHT_CHECK;
            }
            else
            {
                life_driving.drive_state = life_driving.past_drvie_state;
            }
        }
    }
}

void QNode::goalStateCallback(const move_base_msgs::MoveBaseActionResult &goal_arrival_status)
{
    if(goal_arrival_status.status.status == 3)
    {
        std::cout << "arrival !!!" << std::endl;
        life_driving.tunnel_cnt = 0;
        life_driving.arrive_goal = true;
    }
    else if(goal_arrival_status.status.status == 4)
    {
//        std::cout << "republish !!!" << std::endl;

//        goal_pos_y += 0.03;
//        goal_pos_x -= 0.03;

//        goal_pose.header.frame_id = "map";
//        goal_pose.header.stamp = ros::Time::now();
//        goal_pose.pose.orientation.w = 0.7092063831855171;
//        goal_pose.pose.orientation.z = -0.7050009262752196;
//        goal_pose.pose.position.x = goal_pos_x;
//        goal_pose.pose.position.y = goal_pos_y;
//        goal_pose_pub.publish(goal_pose);
//        msleep(10);


        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.pose.pose.orientation.w = imu_w;
        initial_pose.pose.pose.orientation.z = imu_z;
        initial_pose.pose.pose.position.x = goal_pos_x;
        initial_pose.pose.pose.position.y = goal_pos_y;

        initial_pose_pub.publish(initial_pose);
        initial_pose_pub.publish(initial_pose);
        initial_pose_pub.publish(initial_pose);
        msleep(100);

//        life_driving.arrive_goal = true;
    }
}

void QNode::laserMsgCallback(const sensor_msgs::LaserScan &laser_arr)
{
    std::vector<double> distances;

    distances.push_back(laser_arr.ranges.at(0));

    for(int i = 0 ; i < 5 ; i++)
    {
        distances.push_back(laser_arr.ranges.at(1 + i));
        distances.push_back(laser_arr.ranges.at(359 - i));
    }

    double meanDistance = 0.0;
    int    nValidDistance = 0;

    for(int i = 0; i < distances.size(); i++)
    {
        if(distances[i]> 0.001)
        {
            meanDistance += distances[i];
            nValidDistance++;
        }
    }

    if(nValidDistance != 0)
    {
        meanDistance /= (double)nValidDistance;

        life_driving.lidarDistance = meanDistance;
#if CHECK_LIDAR
        cout <<"meanDistance: " << meanDistance << endl;
        cout <<"obstcle close" << endl;
#endif

        if(meanDistance < 0.275)
        {
            std::cout <<"im fuck u : "<< my_imu << std::endl;
            std::cout <<"diff im fuck u : "<< fabs(my_imu - 90.) << std::endl;
            if(fabs(my_imu - 90.) < 15.0 && life_driving.drive_state == LifeDriving::TURTLE_GO)
            {
                cout <<"obstcle " << endl;
                life_driving.bObstacleDetected = true;
                //life_driving.drive_state  = LifeDriving::TURTLE_AVOID_OBSTACLE;
                return ;
            }
        }
    }

    if(life_driving.drive_state == LifeDriving::TURTLE_AVOID_OBSTACLE)
        life_driving.bObstacleDetected = false;
}

void QNode::footPrintMsgCallback(const geometry_msgs::PolygonStamped &foot_print)
{
    static unsigned int X = 0;
    static unsigned int Y = 0;
    static bool initFootPrint = false;

    if(initFootPrint == false)
    {
        past_foot_print[0][X] = foot_print.polygon.points[0].x;
        past_foot_print[0][Y] = foot_print.polygon.points[0].y;
        past_foot_print[1][X] = foot_print.polygon.points[1].x;
        past_foot_print[1][Y] = foot_print.polygon.points[1].y;
        past_foot_print[2][X] = foot_print.polygon.points[2].x;
        past_foot_print[2][Y] = foot_print.polygon.points[2].y;
        past_foot_print[3][X] = foot_print.polygon.points[3].x;
        past_foot_print[3][Y] = foot_print.polygon.points[3].y;

        initFootPrint = true;
    }
    else
    {
        diff_foot_print[0][X] = fabs(foot_print.polygon.points[0].x - past_foot_print[0][X]);
        diff_foot_print[0][Y] = fabs(foot_print.polygon.points[0].y - past_foot_print[0][Y]);
        diff_foot_print[1][X] = fabs(foot_print.polygon.points[1].x - past_foot_print[1][X]);
        diff_foot_print[1][Y] = fabs(foot_print.polygon.points[1].y - past_foot_print[1][Y]);
        diff_foot_print[2][X] = fabs(foot_print.polygon.points[2].x - past_foot_print[2][X]);
        diff_foot_print[2][Y] = fabs(foot_print.polygon.points[2].y - past_foot_print[2][Y]);
        diff_foot_print[3][X] = fabs(foot_print.polygon.points[3].x - past_foot_print[3][X]);
        diff_foot_print[3][Y] = fabs(foot_print.polygon.points[3].y - past_foot_print[3][Y]);

        past_foot_print[0][X] = foot_print.polygon.points[0].x;
        past_foot_print[0][Y] = foot_print.polygon.points[0].y;
        past_foot_print[1][X] = foot_print.polygon.points[1].x;
        past_foot_print[1][Y] = foot_print.polygon.points[1].y;
        past_foot_print[2][X] = foot_print.polygon.points[2].x;
        past_foot_print[2][Y] = foot_print.polygon.points[2].y;
        past_foot_print[3][X] = foot_print.polygon.points[3].x;
        past_foot_print[3][Y] = foot_print.polygon.points[3].y;

        std::cout << " diff_foot_print[0][X] = " <<  diff_foot_print[0][X] << std::endl;
        std::cout << " diff_foot_print[0][X] = " <<  diff_foot_print[0][Y] << std::endl;
        std::cout << " diff_foot_print[1][X] = " <<  diff_foot_print[1][X] << std::endl;
        std::cout << " diff_foot_print[1][X] = " <<  diff_foot_print[1][Y] << std::endl;
        std::cout << " diff_foot_print[2][X] = " <<  diff_foot_print[2][X] << std::endl;
        std::cout << " diff_foot_print[2][X] = " <<  diff_foot_print[2][Y] << std::endl;
        std::cout << " diff_foot_print[3][X] = " <<  diff_foot_print[3][X] << std::endl;
        std::cout << " diff_foot_print[3][X] = " <<  diff_foot_print[3][Y] << std::endl;
    }
}

void QNode::stop()
{
    life_driving.setVelocity(0.0, 0.0);
    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
}

void QNode::tunnelInit()
{
//    past_foot_print.polygon.points[0].x = 0.0;
//    past_foot_print.polygon.points[0].y = 0.0;
//    past_foot_print.polygon.points[1].x = 0.0;
//    past_foot_print.polygon.points[1].y = 0.0;
//    past_foot_print.polygon.points[2].x = 0.0;
//    past_foot_print.polygon.points[2].y = 0.0;
//    past_foot_print.polygon.points[3].x = 0.0;
//    past_foot_print.polygon.points[3].y = 0.0;

    std::cout <<" tunnel " << std::endl;
    life_driving.is_in_tunnel = true;
//    life_driving.setVelocity(0.22, 1.5);
//    cmd_vel_pub.publish(life_driving.cmd_vel_msg);
//    msleep(500);


//    life_driving.setVelocity(0.0, 0.0);
//    cmd_vel_pub.publish(life_driving.cmd_vel_msg);

//    msleep(100);
//    initial_pose.header.frame_id = "map";
//    initial_pose.header.stamp = ros::Time::now();
//    initial_pose.pose.pose.orientation.w = imu_w;
//    initial_pose.pose.pose.orientation.z = imu_z;
//    initial_pose.pose.pose.position.x = 0.05;
//    initial_pose.pose.pose.position.y = 0.05;

//    initial_pose_pub.publish(initial_pose);
//    initial_pose_pub.publish(initial_pose);
//    initial_pose_pub.publish(initial_pose);
//    msleep(2000);
    stop();
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose.orientation.w = 0.7092063831855171;
    goal_pose.pose.orientation.z = -0.7050009262752196;
    goal_pose.pose.position.x = goal_pos_x;
    goal_pose.pose.position.y = goal_pos_y;
    goal_pose_pub.publish(goal_pose);
    goal_pose_pub.publish(goal_pose);
    goal_pose_pub.publish(goal_pose);
    msleep(1000);
}

void QNode::escapeTunnel()
{
    if(life_driving.arrive_goal && tunnelEscapeState == TUNNEL_GO)
    {
        if(fabs(life_driving.lidarDistance - 0.192) < 0.02 && life_driving.lidarDistance != 0.0)
        {
            tunnelEscapeState = TUNNEL_TURN;
        }
        else if((life_driving.lidarDistance - 0.192) < 0.0)
        {
            life_driving.setVelocity(-0.10, 0.0);
        }
        else if((life_driving.lidarDistance - 0.192) > 0.0)
        {
            life_driving.setVelocity(0.12, 0.0);
        }
        cmd_vel_pub.publish(life_driving.cmd_vel_msg);

//        std::cout << "TUNNEL_GO" << std::endl;
//        life_driving.setVelocity(0.22, 0.0);
//        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
//        msleep(1500);
//        life_driving.setVelocity(-0.22, 0.0);
//        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
//        msleep(350);
//        life_driving.setVelocity(0.0, 0.0);
//        cmd_vel_pub.publish(life_driving.cmd_vel_msg);
//        msleep(1);

    }
    else if(life_driving.arrive_goal && tunnelEscapeState == TUNNEL_TURN)
    {
        std::cout << "TUNNEL_GO" << std::endl;
        if((my_imu < 358.0 && my_imu >= 180.0) || (my_imu > 2.0 && my_imu < 180.0))
        {
            life_driving.setVelocity(0.0, escape_imu * TURN_GAIN * 2.0);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);
        }
        else
        {
            life_driving.setVelocity(0.0, 0.0);
            cmd_vel_pub.publish(life_driving.cmd_vel_msg);
            tunnelEscapeState = TUNNEL_ESCAPE;
        }
    }
    else
    {
//        std::cout<< "past_foot_print.polygon.points[0].x = " << past_foot_print.polygon.points[0].x<< std::endl;
//        static int noEyeState = 0;
//        if(fabs(foot_print.polygon.points[0].x - past_foot_print.polygon.points[0].x) < 0.01 && fabs(foot_print.polygon.points[0].y - past_foot_print.polygon.points[0].y) < 0.01 &&
//            fabs(foot_print.polygon.points[1].x - past_foot_print.polygon.points[1].x) < 0.01 && fabs(foot_print.polygon.points[1].y - past_foot_print.polygon.points[1].y) < 0.01 &&
//            fabs(foot_print.polygon.points[2].x - past_foot_print.polygon.points[2].x) < 0.01 && fabs(foot_print.polygon.points[2].y - past_foot_print.polygon.points[2].y) < 0.01 &&
//            fabs(foot_print.polygon.points[3].x - past_foot_print.polygon.points[3].x) < 0.01 && fabs(foot_print.polygon.points[3].y - past_foot_print.polygon.points[3].y) < 0.01)
//        {
//            noEyeState++;
//        }
//        else
//        {
//            noEyeState = 0;
//        }
//        past_foot_print = foot_print;

//        std::cout << "noEyeState = " << noEyeState << std::endl;
    }
}

bool QNode::turnToOtherSide()
{
    //    if(life_driving.acrossDirection == ACROSS_LEFT)
    //    {
    //        if(my_imu < 235.0)
    //            life_driving.setVelocity(0.0, 1.5);
    //        else
    //        {
    //            return false;
    //        }
    //    }
    //    else
    //    {
    //        if(my_imu > 125.0)
    //        {
    //            life_driving.setVelocity(0.0, -1.5);
    //        }
    //        else
    //        {
    //            return false;
    //        }
    //    }
    if(life_driving.acrossDirection == ACROSS_LEFT)
    {
        if(my_imu < 235.0)
            life_driving.setVelocity(0.22, 1.8);
        else
        {
            return false;
        }
    }
    else
    {
        if(my_imu > 135.0)
        {
            life_driving.setVelocity(0.22, -1.8);
        }
        else
        {
            return false;
        }
    }
    //    return true;
    //right 90~100 x right
    //left 70~80 x left


    //    const int IMG_END_X     = 320;
    //    const int IMG_BOUNDARY  = 32;

    //    if(life_driving.acrossDirection == ACROSS_LEFT)
    //    {

    //        if((life_driving.left_line_start.x + life_driving.left_line_end.x)/2 < IMG_END_X/2 &&
    //            (life_driving.left_line_degree >= 55.0 && life_driving.left_line_degree <= 80.0) &&
    //             life_driving.bRightLineDetected == false   )
    //        {
    //            return false;
    //        }
    //        else
    //        {
    //            life_driving.setVelocity(0.22, 1.9);
    //        }

    ////        if(life_driving.left_line_start.x == 0)
    ////            life_driving.setVelocity(0.22, 1.9);
    ////        else
    ////        {
    ////            return false;
    ////        }
    //    }
    //    else
    //    {
    //        if((life_driving.right_line_start.x + life_driving.right_line_end.x)/2> IMG_END_X/2  &&
    //            (life_driving.right_line_degree >= 90.0 && life_driving.right_line_degree <= 115.0) &&
    //             life_driving.bLeftLineDetected == false)
    //        {
    //            return false;
    //        }
    //        else
    //        {
    //            life_driving.setVelocity(0.22, -1.9);
    //        }


    ////        if(life_driving.right_line_start.x == 0)
    ////        {
    ////            life_driving.setVelocity(0.22, -1.9);
    ////        }
    ////        else
    ////        {
    ////            return false;
    ////        }
    //    }

    return true;
}

void QNode::rvizInit()
{
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.pose.pose.orientation.w = imu_w;
    initial_pose.pose.pose.orientation.z = imu_z;
    initial_pose.pose.pose.position.x = init_pos_x;
    initial_pose.pose.pose.position.y = init_pos_y;

    initial_pose_pub.publish(initial_pose);
    initial_pose_pub.publish(initial_pose);
    initial_pose_pub.publish(initial_pose);
    msleep(3000);
}
}  // namespace turtle_master
