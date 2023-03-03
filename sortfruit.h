#ifndef WRITECHARACTER_H
#define WRITECHARACTER_H
#include <QJsonDocument>
#include <QJsonParseError>
#include <QFile>
#include <QJsonObject>
#include <QDebug>
#include <QJsonArray>
#include <Eigen/Dense>
#include "math.h"
#include <iostream>
#include "auborobot.h"
#include <unistd.h>

class SortFruit:public AuboRobot
{
private:
    Eigen::MatrixXd m_wayPoint;
    Eigen::Matrix3d TB;
    Eigen::Matrix3d Place_TB;
    QJsonArray m_strokeIK;

public:
    bool stop_move;
    SortFruit();
    //tramsform among euler ,quaternion and rotation Matrix
    Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);

    Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

    Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);
    Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);

    //重置
    void ClearTrajectory();
    //将路径点存入类内私有变量
    void GetPoint(double x,double y,double z);
    bool HaveTrajectory();

    Eigen::MatrixXd Pick_ComputeSample(double initPos[3],Eigen::Quaterniond quaternion,float info[9]);
    bool Pick_ComputeIK();
    int Pick_Approach();
    int Pick_Approach_2();
    int Pick_Retreat();

    Eigen::MatrixXd Place_ComputeSample(double initPos[3],Eigen::Quaterniond quaternion,float info[9]);
    bool Place_ComputeIK();
    int Place_Approach();
    int Place_Retreat();


    ~SortFruit();
};

#endif // WRITECHARACTER_H
