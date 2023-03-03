#ifndef AUBOROBOT_H
#define AUBOROBOT_H

#include <string>
#include <Eigen/Core>
#include <QJsonArray>
#include "serviceinterface.h"

#include "common.h"
#include "twist.h"
#include "orientation.h"
#include "kinematicmodel.h"
#include "auborobot.h"

enum AuboRobotType {AuboI_3, AuboI_5, AuboI_7, AuboI_10};

enum CallbackType {CallbackRoadPoint, CallbackJointStatus, CallbackEndSpeed, CallbackRobotEvent};

struct JointMaximum
{
    double vel[6];
    double acc[6];
};

struct EndMaximum
{
    double vel[2];
    double acc[2];
};

class AuboRobot
{
public:
    AuboRobot();
    ~AuboRobot();

    int connect(const std::string & ip_address = "192.168.0.2", int port = 8899);
    int disconnect();
    bool isConnected();


    // Set properties.
    // Set and get the joints' maximum accelerations and velocities.关节
    int setJointMaximum(const JointMaximum & joint_maximum);
    int getJointMaximum(JointMaximum & joint_maximum);

    // Set and get the end-effector's maximum acceleartion and velocity.末端执行器
    int setEndMaximum(const EndMaximum & end_maximum);
    int getEndMaximum(EndMaximum & end_maximum);

    // Kinematics.运动学
    void forwardKinematics(Eigen::VectorXd & joint_angle, Eigen::Matrix4d & pose_flange);
    bool inverseKinematics(Eigen::VectorXd & ref_joint_angle, Eigen::Matrix4d & pose_flange, Eigen::VectorXd & joint_angle);

    // Get current state.
    int getJointAngle(Eigen::VectorXd & joint_angle);
    int getFlangePose(Eigen::Matrix4d & flange_pose);
    void getJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & mat_jacob);
    // set collision class
    int set_collision_class(int grade);
    // Motion control functions.
    int move(const Eigen::VectorXd & joint_angle, bool is_blocked);
    int move(const Eigen::Matrix4d & flange_pose, bool is_blocked);
    int moveline(const Eigen::VectorXd & joint_angle,bool is_blocked);
    int movelineF(const Eigen::MatrixXd & flange_pose, bool is_blocked);
    int movecurve(const Eigen::VectorXd & joint_angle1,const Eigen::VectorXd & joint_angle2,const Eigen::VectorXd & joint_angle3,bool is_blocked);
    int setTransparentTransmissionStatus(bool status);
    int set_canbus();
    int transmitTransparently(std::vector<Eigen::VectorXd> & buffer_joint_angle);
    int transmitTransparently(Eigen::VectorXd & joint_angle);
    int transmitTransparently(Eigen::Matrix4d & flange_pose);
    int getBufferDataSize(int & data_size);
//    int Move_Rot(double ro[3],double ang);
//    int Move_Line(int a, float b);
//    int Move_Line_Press(int a,float b);
//    int Jointone_move(float ang);
//    int Jointtwo_move(float ang);
//    int Jointthree_move(float ang);
//    int Jointfour_move(float ang);
//    int Jointfive_move(float ang);
//    int Jointsix_move(float ang);

    //0316

    int teach_move(aubo_robot_namespace::teach_mode mode,bool direction);
    int teach_stop();
    int startup();
    int shutdown();
    int stop();
    int pause();
    int resume();
    int fastStop();
    int collisionRecover();

    //
    int getRobotState(aubo_robot_namespace::RobotState & state);
    int setWorkMode(aubo_robot_namespace::RobotWorkMode mode);
    int getWorkMode(aubo_robot_namespace::RobotWorkMode & mode);
    int getDeviceInfo(aubo_robot_namespace::RobotDevInfo & device_info);

    // Set the properties of IO pins on the flange.
    int setIoProperty();
    int getIoProperty();

    // Register callback functions.
    int registerCallbackFunction(void * ptr_function, void * arg, CallbackType type);
    int setCallbackStatus(bool enable, CallbackType type);


private:
    AuboRobotType robot_type_;
    ServiceInterface robotService;

    std::vector<double> len_params_;
    std::vector<xar::Twist> twists_;
    Eigen::Matrix4d pose_flange_home_;
    void setConfiguration();
    bool checkSolutionValid(const Eigen::Matrix4d & pose_flange_ref, const Eigen::VectorXd & joint_angles, double eps_pos = 5.0, double eps_orient = xar::deg2rad(5.0));
};

#endif // AUBOROBOT_H
