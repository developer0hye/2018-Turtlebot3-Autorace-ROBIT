#include "../include/turtle_master/life_driving.hpp"

int MIDDLE_OF_IMAGE =   162; // center: 162     -: white line  +: yellow line

#define MIDDEL_DEGREE               90.0
#define START_RIGHT_CURVE           275
#define START_LEFT_CURVE            50

#define MARGIN_OF_STRAIGHT          28
#define MARGIN_OF_STRAIGHT_DEGREE   7

#define STRAIGHT_GAIN               0.014
#define CURVE_GAIN                  0.017
LifeDriving::LifeDriving(QObject *parent) :
    QObject(parent),
    isParkingComplete(false),
    imu_yaw(0.0),
    tunnel_cnt(350),
    is_in_tunnel(false),
    arrive_goal(false),
    traffic_light_cnt(1000),
    cds_val(0),
    psd_val(0),
    acrossState(ACROSS_NORMAL),
    acrossDirection(0),
    parking_mode(false),
    tracking_point(0),
    bObstacleDetected(false),
    isObstacleMissionStart(false),
    avoidMissionState(TURN),
    obstacle_distance(0.0),
    parkingDrivingMode(0),
    laser_center(0.0),
    avoidDirection(LEFT),
    parkingPosition(RIGHT),
    drive_state(TURTLE_STOP),
    past_drvie_state(TURTLE_STOP),
    lidarDistance(0.0),
    lidarRightDistance(0.0)
{
    drivingTimer = new QTimer(this);
    connect(drivingTimer, SIGNAL(timeout()), this, SLOT(parkingMotion()));
    drivingTimer->start(10);
}

void LifeDriving::setLineInformation(const turtle_vision::vision_msg &vision_msg)
{
    this->right_line_degree = vision_msg.right_line_degree;
    this->left_line_degree = vision_msg.left_line_degree;

    this->right_line_start.x = vision_msg.right_line_start_X;
    this->right_line_start.y = vision_msg.right_line_start_Y;

    this->right_line_end.x = vision_msg.right_line_end_X;
    this->right_line_end.y = vision_msg.right_line_end_Y;


    this->left_line_start.x = vision_msg.left_line_start_X;
    this->left_line_start.y = vision_msg.left_line_start_Y;

    this->left_line_end.x = vision_msg.left_line_end_X;
    this->left_line_end.y = vision_msg.left_line_end_Y;

    if(this->right_line_start.x == 0 && this->right_line_start.y == 0 &&
            this->right_line_end.x == 0 && this->right_line_end.y == 0)
        this->bRightLineDetected = false;
    else
        this->bRightLineDetected = true;

    if(this->left_line_start.x == 0 && this->left_line_start.y == 0 &&
            this->left_line_end.x == 0 && this->left_line_end.y == 0)
        this->bLeftLineDetected = false;
    else
        this->bLeftLineDetected = true;
}

void LifeDriving::setTrackingPoint(const turtle_vision::vision_msg &vision_msg)
{
    this->tracking_point = vision_msg.tracking_point;
}

void LifeDriving::turtlebot3_velocity_update()
{
    if(tracking_point != 0)
    {
        static double past_tracking_point = tracking_point;

        double yawVelocity = (double)(162 - tracking_point) * (STRAIGHT_GAIN) *1.0;
        setVelocity(0.22, yawVelocity);


        past_tracking_point = tracking_point;
    }
    else
    {
        const double f1 = 0.35;
        const double f2 = 1. - f1;

        if(this->bRightLineDetected && this->bLeftLineDetected)
        {
            double middlePoint = 0.0;
            static double yawVelocity = 0.0;
            double yawVcur = 0.0;

            middlePoint = (right_line_start.x + left_line_start.x)/2.0;

            yawVcur = ((double)MIDDLE_OF_IMAGE - middlePoint) * STRAIGHT_GAIN;

            yawVelocity = f2 * yawVcur +  f1 * yawVelocity;

            setVelocity(0.22, yawVelocity);
        }
        else if(this->bRightLineDetected && !this->bLeftLineDetected)
        {
            static double yawVelocity = 0.0;
            double yawVcur = 0.0;

            if(right_line_start.x < START_RIGHT_CURVE)
                yawVcur = fabs((double)START_RIGHT_CURVE - right_line_start.x) * CURVE_GAIN;

            yawVelocity = f2 * yawVcur + f1 *yawVelocity;

            setVelocity(0.22, yawVelocity);
        }
        else if(!this->bRightLineDetected && this->bLeftLineDetected)
        {
            static double yawVelocity = 0.0;
            double yawVcur = 0.0;

            if(left_line_start.x > START_LEFT_CURVE)
                yawVcur = - fabs((double)START_LEFT_CURVE - left_line_start.x) * CURVE_GAIN;

            yawVelocity = f2 * yawVcur + f1 *yawVelocity;

            setVelocity(0.22, yawVelocity);
        }
    }
}

void LifeDriving::setVelocity(double linear, double screw)
{
    cmd_vel_msg.linear.x = linear;
    cmd_vel_msg.angular.z = screw;
}

void LifeDriving::parkingMotion()
{
    traffic_light_cnt ++;
    tunnel_cnt ++;

//    if(is_in_tunnel && arrive_goal)
//    {
//        if(tunnel_cnt < 300)
//            setVelocity(0.22, 0.0);
//        else if(tunnel_cnt < 345)
//            setVelocity(-0.22, 0.0);
//        else
//        {
//            if(!cds_val)
//            {
//                is_in_tunnel = false;
//                arrive_goal = false;
//                setVelocity(0.0, 0.0);
//                std::cout << "escape !!! " << std::endl;
//            }
//            else
//            {
//                if(imu_yaw < 0.0)
//                    setVelocity(0.0, 0.75);
//                else if(imu_yaw > 2.0)
//                    setVelocity(0.0, -0.75);
//                else
//                {
//                    setVelocity(0.20, 0.0);
//                }
//            }
//        }
//    }
}
