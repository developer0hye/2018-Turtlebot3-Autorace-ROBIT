#ifndef LIFE_DRIVING_H
#define LIFE_DRIVING_H

#include <iostream>
#include <QObject>
#include <QTimer>
#include <geometry_msgs/Twist.h>

#include "../../../devel/include/turtle_vision/vision_msg.h"



typedef struct Point{
    int x;
    int y;
}Point;

class LifeDriving: public QObject{

    Q_OBJECT
public:
    enum DRIVING_STATE{TURTLE_STOP,
                       TURTLE_TRAFFIC_LIGHT_CHECK,
                       TURTLE_GO,
                       TURTLE_ACROSS,
                       TURTLE_STOP_POGBA,
                       TURTLE_AVOID_OBSTACLE,
                       TURTLE_PARK, //PARKING
                       TURTLE_TUNNEL//OBSTACLE IN TUNNEL
                      };

    explicit LifeDriving(QObject *parent = 0);

    /*line*/
    void setLineInformation(const turtle_vision::vision_msg &vision_msg);

    double right_line_degree;
    double left_line_degree;

    Point right_line_start;
    Point right_line_end;
    Point left_line_start;
    Point left_line_end;

    bool bRightLineDetected;//is_right_line;
    bool bLeftLineDetected;//is_left_line;

    /*line end*/

    /*tracking_point*/
    void setTrackingPoint(const turtle_vision::vision_msg &vision_msg);
    void setTrackingPoint(const int point){tracking_point = point;}
    int tracking_point;
    /*tracking_point end*/

    DRIVING_STATE drive_state;
    DRIVING_STATE past_drvie_state;

    int across_target_point;

    int traffic_light_cnt;
    int tunnel_cnt;

    int32_t cds_val;
    int32_t psd_val;

    double imu_yaw;
    bool isParkingComplete;
    bool parking_mode;

    int acrossState;
    int acrossDirection;
    enum {ACROSS_NORMAL, ACROSS_DEPARTURE, ACROSS_ARRIVAL, ACROSS_COMPLETE};

    bool is_in_tunnel;
    bool arrive_goal;
    QTimer *drivingTimer;
    geometry_msgs::Twist cmd_vel_msg;
    void setVelocity(double linear, double screw);
    void imuMsgCallBack();

    bool bObstacleDetected;
    bool isObstacleMissionStart;
    int  avoidMissionState;
    int  avoidDirection;
    double obstacle_distance;
    double laser_center;

    double lidarDistance;
    double lidarRightDistance;
    enum {TURN, GO, OBSTACLE_COMPLETE};
    enum {LEFT, RIGHT};

    int parkingDrivingMode;

    int parkingPosition;

public Q_SLOTS:
    void turtlebot3_velocity_update();
    void parkingMotion();
private:

};

#endif // LIFE_DRIVING_H
