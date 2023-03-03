#include "common.h"
#include "orientation.h"
#include "kinematicmodel.h"
#include "auborobot.h"

AuboRobot::AuboRobot() : robot_type_(AuboI_5)
{
    setConfiguration();
}

AuboRobot::~AuboRobot()
{
    setTransparentTransmissionStatus(false);
}

int AuboRobot::connect(const std::string & ip_address, int port)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(ip_address.c_str(), port, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
        return 0;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
        return 1;
    }
}

int AuboRobot::disconnect()
{
    return robotService.robotServiceLogout();
}

bool AuboRobot::isConnected()
{
    bool is_connected;
    robotService.robotServiceGetConnectStatus(is_connected);
    return is_connected;
}

// Set properties.
// Set and get the joints' maximum accelerations and velocities.
int AuboRobot::setJointMaximum(const JointMaximum & joint_maximum)
{
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for (int idx = 0; idx < 6; ++idx)
    {
        jointMaxVelc.jointPara[idx] = joint_maximum.vel[idx];
        jointMaxAcc.jointPara[idx] = joint_maximum.acc[idx];
    }

    int status_acc = robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    int status_vel = robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    if ((0 == status_vel) && (0 == status_acc))
        return 0;
    else
        return 1;
}

int AuboRobot::getJointMaximum(JointMaximum & joint_maximum)
{
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    robotService.robotServiceGetGlobalMoveJointMaxAcc(jointMaxAcc);
    robotService.robotServiceGetGlobalMoveJointMaxVelc(jointMaxVelc);

    for (int idx = 0; idx < 6; ++idx)
    {
        joint_maximum.vel[idx] = jointMaxVelc.jointPara[idx];
        joint_maximum.acc[idx] = jointMaxAcc.jointPara[idx];
    }
    return 0;
}

// Set and get the end-effector's maximum acceleartion and velocity.
int AuboRobot::setEndMaximum(const EndMaximum & end_maximum)
{
    int status_lin_vel = robotService.robotServiceSetGlobalMoveEndMaxLineVelc(end_maximum.vel[0]);
    int status_lin_acc = robotService.robotServiceSetGlobalMoveEndMaxLineAcc(end_maximum.acc[0]);
    int status_ang_vel = robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(end_maximum.vel[1]);
    int status_ang_acc = robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(end_maximum.acc[1]);

    if ((0 == status_lin_vel) && (0 == status_lin_acc) &&
            (0 == status_ang_vel) && (0 == status_ang_acc))
        return 0;
    else
        return 1;
}

int AuboRobot::getEndMaximum(EndMaximum & end_maximum)
{
    double max_lin_vel;
    double max_lin_acc;
    double max_ang_vel;
    double max_ang_acc;
    robotService.robotServiceGetGlobalMoveEndMaxLineVelc(max_lin_vel);
    robotService.robotServiceGetGlobalMoveEndMaxLineAcc(max_lin_acc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(max_ang_vel);
    robotService.robotServiceGetGlobalMoveEndMaxAngleAcc(max_ang_acc);

    end_maximum.vel[0] = max_lin_vel;
    end_maximum.vel[1] = max_ang_vel;
    end_maximum.acc[0] = max_lin_acc;
    end_maximum.acc[1] = max_ang_acc;
    return 0;
}

int AuboRobot::getJointAngle(Eigen::VectorXd & joint_angle)
{
    joint_angle = Eigen::MatrixXd::Zero(6,1);

    aubo_robot_namespace::wayPoint_S way_point_s;
    int status = robotService.robotServiceGetCurrentWaypointInfo(way_point_s);
    if (0 != status)
    {
        return 1;
    }
    else
    {
        for (int idx = 0; idx < 6; ++idx)
            joint_angle[idx] = way_point_s.jointpos[idx];

        return 0;
    }
}

int AuboRobot::set_collision_class(int grade)
{
    return robotService.robotServiceSetRobotCollisionClass(grade);
}

int AuboRobot::getFlangePose(Eigen::Matrix4d & flange_pose)
{
    flange_pose = Eigen::Matrix4d::Identity();

    aubo_robot_namespace::wayPoint_S way_point_s;
    int status = robotService.robotServiceGetCurrentWaypointInfo(way_point_s);
    if (0 != status)
    {
        return 1;
    }
    else
    {
        flange_pose(0, 3) = way_point_s.cartPos.position.x;
        flange_pose(1, 3) = way_point_s.cartPos.position.y;
        flange_pose(2, 3) = way_point_s.cartPos.position.z;
        xar::Quaternion quat;
        quat.w() = way_point_s.orientation.w;
        quat.vec() = Eigen::Vector3d(way_point_s.orientation.x, way_point_s.orientation.y, way_point_s.orientation.z);
        flange_pose.block(0, 0, 3, 3) = quat.toRotationMatrix();

        return 0;
    }
}


// Motion control functions.
int AuboRobot::move(const Eigen::VectorXd & joint_angle, bool is_blocked)
{
    Eigen::VectorXd joint_angle_copy = joint_angle;
    return robotService.robotServiceJointMove(joint_angle_copy.data(),is_blocked);
}

int AuboRobot::moveline(const Eigen::VectorXd & joint_angle, bool is_blocked)
{
//    std::cout<<"begin line move"<<std::endl;
//    Eigen::VectorXd angle(6,1);
//    getJointAngle(angle);
    Eigen::VectorXd joint_angle_copy = joint_angle;
//    robotService.robotServiceJointMove(angle.data(),is_blocked);
    return robotService.robotServiceLineMove(joint_angle_copy.data(), is_blocked);
}

int AuboRobot::movelineF(const Eigen::MatrixXd & flange_pose, bool is_blocked)
{

    Eigen::Matrix4d flange_pose_copy = flange_pose;
    Eigen::VectorXd joint_angle_refer,joint_angel;
    getJointAngle(joint_angle_refer);
    inverseKinematics(joint_angle_refer,flange_pose_copy,joint_angel) ;
    return moveline(joint_angel,is_blocked) ;
}

int AuboRobot::movecurve(const Eigen::VectorXd & joint_angle1,const Eigen::VectorXd & joint_angle2,const Eigen::VectorXd & joint_angle3,bool is_blocked)
{
    Eigen::VectorXd point1 = joint_angle1;
    Eigen::VectorXd point2 = joint_angle2;
    Eigen::VectorXd point3 = joint_angle3;
    std::cout<<"begin curve move"<<std::endl;
    robotService.robotServiceAddGlobalWayPoint(point1.data());
    robotService.robotServiceAddGlobalWayPoint(point2.data());
    robotService.robotServiceAddGlobalWayPoint(point3.data());
    robotService.robotServiceSetGlobalCircularLoopTimes(0);
    robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR,is_blocked);
    robotService.robotServiceClearGlobalWayPointVector();
}
int AuboRobot::move(const Eigen::Matrix4d &flange_pose, bool is_blocked)
{
    Eigen::Matrix4d flange_pose_copy = flange_pose;
    Eigen::VectorXd joint_angle_refer;
    getJointAngle(joint_angle_refer);

    Eigen::VectorXd joint_angle;
    inverseKinematics(joint_angle_refer, flange_pose_copy, joint_angle);

    return move(Eigen::VectorXd(joint_angle), is_blocked);
}

int AuboRobot::setTransparentTransmissionStatus(bool status)
{
    if (status)
        return robotService.robotServiceEnterTcp2CanbusMode();
    else
        return robotService.robotServiceLeaveTcp2CanbusMode();
}

int AuboRobot::transmitTransparently(std::vector<Eigen::VectorXd> & buffer_joint_angle)
{
    for (unsigned int idx = 0; idx < buffer_joint_angle.size(); ++idx)
        if (0 != robotService.robotServiceSetRobotPosData2Canbus(buffer_joint_angle[idx].data()))
            return 1;

    return 0;
}
int AuboRobot::set_canbus()
{
    return  robotService.robotServiceEnterTcp2CanbusMode();
}
int AuboRobot::transmitTransparently(Eigen::VectorXd & joint_angle)
{
    return robotService.robotServiceSetRobotPosData2Canbus(joint_angle.data());
}
int AuboRobot::transmitTransparently(Eigen::Matrix4d & flange_pose)
{
    Eigen::Matrix4d flange_pose_copy = flange_pose;
    Eigen::VectorXd joint_angle_refer;
    getJointAngle(joint_angle_refer);

    Eigen::VectorXd joint_angle;
    inverseKinematics(joint_angle_refer, flange_pose_copy, joint_angle);

    return robotService.robotServiceSetRobotPosData2Canbus(joint_angle.data());
}
int AuboRobot::getBufferDataSize(int & data_size)
{
    aubo_robot_namespace::RobotDiagnosis diagnosis;
    if (0 == robotService.robotServiceGetRobotDiagnosisInfo(diagnosis))
    {
        data_size = diagnosis.macTargetPosDataSize;
        return 0;
    }
    else
    {
        return 1;
    }
}

int AuboRobot::startup()
{
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    toolDynamicsParam.payload=8;
    int ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }
    robotService.robotServiceInitGlobalMoveProfile();
    setJointMaximum({{0.1,0.1,0.1,0.1,0.1,0.1},{0.35,0.35,0.35,0.3,0.3,0.3}});
    setEndMaximum({{0.1,0.1},{0.1,0.1}});
}

int AuboRobot::shutdown()
{
    return robotService.robotServiceRobotShutdown();
}

int AuboRobot::stop()
{
    return robotService.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
}

int AuboRobot::pause()
{
    return robotService.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMovePause);
}

int AuboRobot::resume()
{
    return robotService.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveContinue);
}

int AuboRobot::fastStop()
{
    return robotService.robotMoveFastStop();
}

int AuboRobot::collisionRecover()
{
    return robotService.robotServiceCollisionRecover();
}


// Kinematics.
void AuboRobot::forwardKinematics(Eigen::VectorXd & joint_angle, Eigen::Matrix4d & pose_flange)
{
    pose_flange = xar::prodOfExp(twists_, joint_angle, pose_flange_home_);
}

bool AuboRobot::inverseKinematics(Eigen::VectorXd & joint_angle_ref, Eigen::Matrix4d & pose_flange, Eigen::VectorXd & joint_angle)
{
    double len_1 = len_params_[0];
    double len_2 = len_params_[1];
    double len_3 = len_params_[2];
    double len_4 = len_params_[3];
    double len_5 = len_params_[4];
    double len_6 = len_params_[5];
    double len_7 = len_params_[6];
    double len_8 = len_params_[7];

    int valid_count = 0;
    Eigen::MatrixXd valid_solutions = Eigen::MatrixXd::Zero(6,8);

    // Step 1: Calculate theta-1 which has two solutions.
    Eigen::Vector3d pos_flange = pose_flange.block(0,3,3,1);
    Eigen::Matrix3d orient_flange = pose_flange.block(0,0,3,3);
    Eigen::Vector3d pos_aux = pos_flange - len_8*pose_flange.block(0,2,3,1);
    double coeff_a = -len_2 + len_4 - len_6;
    double coeff_b_1 = xar::sqrt(std::pow(pos_aux(0),2) + std::pow(pos_aux(1),2) - std::pow(coeff_a,2));
    double coeff_b_2 = -xar::sqrt(std::pow(pos_aux(0),2) + std::pow(pos_aux(1),2) - std::pow(coeff_a,2));
    double theta_1_1 = std::atan2(-(coeff_a*pos_aux(0)+coeff_b_1*pos_aux(1)), coeff_a*pos_aux(1)-coeff_b_1*pos_aux(0));
    double theta_1_2 = std::atan2(-(coeff_a*pos_aux(0)+coeff_b_2*pos_aux(1)), coeff_a*pos_aux(1)-coeff_b_2*pos_aux(0));

    // Step 2: Calculate theta_5. --> Two solutions of theta_5 for each theta_1.
    double EPS = 1E-12;
    double value_temp = (pos_flange(0)*std::sin(theta_1_1) - pos_flange(1)*std::cos(theta_1_1) - (len_2-len_4+len_6))/len_8;
    if (std::abs(value_temp) < 1.0+EPS)
    {
        double theta_5_1_1 = xar::acos(value_temp);
        double theta_5_1_2 = -xar::acos(value_temp);

        // Step 3: Calculate theta_6.
        // If sin(theta_5) == 0, theta_6 can be arbitrary value.
        // Hence, decide it to be the corresponding joint's angle in the reference.
        double theta_6_1_1;
        if (std::abs(sin(theta_5_1_1)) < EPS)
        {
            theta_6_1_1 = joint_angle_ref(5);
        }
        else
        {
            double coord_y = (orient_flange(0,1)*std::sin(theta_1_1)-orient_flange(1,1)*std::cos(theta_1_1)) * std::sin(theta_5_1_1);
            double coord_x = (orient_flange(1,0)*std::cos(theta_1_1)-orient_flange(0,0)*std::sin(theta_1_1)) * std::sin(theta_5_1_1);
            theta_6_1_1 = std::atan2(coord_y, coord_x);
        }

        // Step 4: Calculate theta_3.  --> Two solutions for each pair of theta_1 and theta_5.
        // Firstly, we need to calculate 'theta_234 = theta_2-theta_3+theta_4'.
        Eigen::Matrix4d g_234_1_1 = xar::invTransf(xar::expMap(twists_[0], theta_1_1)) * pose_flange *
                xar::invTransf(xar::expMap(twists_[4],theta_5_1_1)*xar::expMap(twists_[5],theta_6_1_1)*pose_flange_home_);
        xar::ExpCoord rot_234_1 = xar::ExpCoord(Eigen::Matrix3d(g_234_1_1.block(0,0,3,3)));
        double theta_234_1_1 = rot_234_1.axis().dot(twists_[1].omg()) * rot_234_1.angle();

        // theta_234 is the composite effect of joint 2, 3, and 4, so it might be out of [-pi, pi].
        double theta_234_refer = joint_angle_ref[1] - joint_angle_ref[2] + joint_angle_ref[4];
        double dist_theta_234 = std::abs(theta_234_1_1-theta_234_refer);
        int num_round = 0;
        for (int idx_round = -2; idx_round <= 2; ++idx_round)
        {
            double dist_theta_234_round = std::abs((theta_234_1_1+double(idx_round)*(2.0*xar::PI)) - theta_234_refer);
            if (dist_theta_234_round < dist_theta_234)
            {
                dist_theta_234 = dist_theta_234_round;
                num_round = idx_round;
            }
        }
        theta_234_1_1 = theta_234_1_1 + double(num_round)*(2.0*xar::PI);

        double temp_m = -(g_234_1_1(0,3) - (len_1+len_3+len_5)*std::sin(theta_234_1_1));
        double temp_n = g_234_1_1(2,3) + (len_1+len_3+len_5)*std::cos(theta_234_1_1) - len_1;
        value_temp = (std::pow(temp_m,2) + std::pow(temp_n,2) - (std::pow(len_3,2)+std::pow(len_5,2))) / (2*len_3*len_5);
        if (std::abs(value_temp) < 1.0+EPS)
        {
            double theta_3_1_1_1 = xar::acos(value_temp);
            double theta_3_1_1_2 = -xar::acos(value_temp);

            // Step 5: Calculate theta_2.
            double coord_y = (len_5*std::cos(theta_3_1_1_1)+len_3)*temp_m + len_5*std::sin(theta_3_1_1_1)*temp_n;
            double coord_x = (len_5*std::cos(theta_3_1_1_1)+len_3)*temp_n - len_5*std::sin(theta_3_1_1_1)*temp_m;
            double theta_2_1_1_1 = std::atan2(coord_y, coord_x);
            coord_y = (len_5*std::cos(theta_3_1_1_2)+len_3)*temp_m + len_5*std::sin(theta_3_1_1_2)*temp_n;
            coord_x = (len_5*std::cos(theta_3_1_1_2)+len_3)*temp_n - len_5*std::sin(theta_3_1_1_2)*temp_m;
            double theta_2_1_1_2 = std::atan2(coord_y, coord_x);

            // Step 6: Calculate theta_4.
            double theta_4_1_1_1 = theta_234_1_1 - theta_2_1_1_1 + theta_3_1_1_1;
            double theta_4_1_1_2 = theta_234_1_1 - theta_2_1_1_2 + theta_3_1_1_2;

            // solution = [theta_1_1; theta_2_1_1_1; theta_3_1_1_1; theta_4_1_1_1; theta_5_1_1; theta_6_1_1];
            Eigen::VectorXd solution(6);
            solution(0) = theta_1_1;
            solution(1) = theta_2_1_1_1;
            solution(2) = theta_3_1_1_1;
            solution(3) = theta_4_1_1_1;
            solution(4) = theta_5_1_1;
            solution(5) = theta_6_1_1;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;

            // solution = [theta_1_1; theta_2_1_1_2; theta_3_1_1_2; theta_4_1_1_2; theta_5_1_1; theta_6_1_1];
            solution(0) = theta_1_1;
            solution(1) = theta_2_1_1_2;
            solution(2) = theta_3_1_1_2;
            solution(3) = theta_4_1_1_2;
            solution(4) = theta_5_1_1;
            solution(5) = theta_6_1_1;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;
        }

        double theta_6_1_2;
        if (std::abs(sin(theta_5_1_2)) < EPS)
        {
            theta_6_1_2 = joint_angle_ref(5);
        }
        else
        {
            double coord_y = (orient_flange(0,1)*std::sin(theta_1_1)-orient_flange(1,1)*std::cos(theta_1_1)) * std::sin(theta_5_1_2);
            double coord_x = (orient_flange(1,0)*std::cos(theta_1_1)-orient_flange(0,0)*std::sin(theta_1_1)) * std::sin(theta_5_1_2);
            theta_6_1_2 = std::atan2(coord_y, coord_x);
        }

        Eigen::Matrix4d g_234_1_2 = xar::invTransf(xar::expMap(twists_[0], theta_1_1)) * pose_flange *
                xar::invTransf(xar::expMap(twists_[4],theta_5_1_2)*xar::expMap(twists_[5],theta_6_1_2)*pose_flange_home_);
        xar::ExpCoord rot_234_2 = xar::ExpCoord(Eigen::Matrix3d(g_234_1_2.block(0,0,3,3)));
        double theta_234_1_2 = rot_234_2.axis().dot(twists_[1].omg()) * rot_234_2.angle();

        dist_theta_234 = std::abs(theta_234_1_2-theta_234_refer);
        num_round = 0;
        for (int idx_round = -2; idx_round <= 2; ++idx_round)
        {
            double dist_theta_234_round = std::abs((theta_234_1_2+double(idx_round)*(2.0*xar::PI)) - theta_234_refer);
            if (dist_theta_234_round < dist_theta_234)
            {
                dist_theta_234 = dist_theta_234_round;
                num_round = idx_round;
            }
        }
        theta_234_1_2 = theta_234_1_2 + double(num_round)*(2.0*xar::PI);

        temp_m = -(g_234_1_2(0,3) - (len_1+len_3+len_5)*std::sin(theta_234_1_2));
        temp_n = g_234_1_2(2,3) + (len_1+len_3+len_5)*std::cos(theta_234_1_2) - len_1;
        value_temp = (std::pow(temp_m,2) + std::pow(temp_n,2) - (std::pow(len_3,2)+std::pow(len_5,2))) / (2*len_3*len_5);
        if (std::abs(value_temp) < 1+EPS)
        {
            double theta_3_1_2_1 = xar::acos(value_temp);
            double theta_3_1_2_2 = -xar::acos(value_temp);

            // Step 5: Calculate theta_2.
            double coord_y = (len_5*std::cos(theta_3_1_2_1)+len_3)*temp_m + len_5*std::sin(theta_3_1_2_1)*temp_n;
            double coord_x = (len_5*std::cos(theta_3_1_2_1)+len_3)*temp_n - len_5*std::sin(theta_3_1_2_1)*temp_m;
            double theta_2_1_2_1 = std::atan2(coord_y, coord_x);

            coord_y = (len_5*std::cos(theta_3_1_2_2)+len_3)*temp_m + len_5*std::sin(theta_3_1_2_2)*temp_n;
            coord_x = (len_5*std::cos(theta_3_1_2_2)+len_3)*temp_n - len_5*std::sin(theta_3_1_2_2)*temp_m;
            double theta_2_1_2_2 = std::atan2(coord_y, coord_x);


            // Step 6: Calculate theta_4.
            double theta_4_1_2_1 = theta_234_1_2 - theta_2_1_2_1 + theta_3_1_2_1;
            double theta_4_1_2_2 = theta_234_1_2 - theta_2_1_2_2 + theta_3_1_2_2;

            // solution = [theta_1_1; theta_2_1_2_1; theta_3_1_2_1; theta_4_1_2_1; theta_5_1_2; theta_6_1_2];
            Eigen::VectorXd solution(6);
            solution(0) = theta_1_1;
            solution(1) = theta_2_1_2_1;
            solution(2) = theta_3_1_2_1;
            solution(3) = theta_4_1_2_1;
            solution(4) = theta_5_1_2;
            solution(5) = theta_6_1_2;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;

            // solution = [theta_1_1; theta_2_1_2_2; theta_3_1_2_2; theta_4_1_2_2; theta_5_1_2; theta_6_1_2];
            solution(0) = theta_1_1;
            solution(1) = theta_2_1_2_2;
            solution(2) = theta_3_1_2_2;
            solution(3) = theta_4_1_2_2;
            solution(4) = theta_5_1_2;
            solution(5) = theta_6_1_2;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;
        }
    }

    // Step 2: Calculate theta_5. --> Two solutions of theta_5 for each theta_1.
    value_temp = (pos_flange(0)*std::sin(theta_1_2) - pos_flange(1)*std::cos(theta_1_2) - (len_2-len_4+len_6)) / len_8;
    if (std::abs(value_temp) < 1.0+EPS)
    {
        double theta_5_2_1 = xar::acos(value_temp);
        double theta_5_2_2 = -xar::acos(value_temp);

        // Step 3: Calculate theta_6.
        // If sin(theta_5) == 0, theta_6 can be arbitrary value. Hence, decide it to be the corresponding joint's angle in the reference.
        double theta_6_2_1;
        if (std::abs(sin(theta_5_2_1)) < EPS)
        {
            theta_6_2_1 = joint_angle_ref(5);
        }
        else
        {
            double coord_y = (orient_flange(0,1)*std::sin(theta_1_2)-orient_flange(1,1)*std::cos(theta_1_2)) * std::sin(theta_5_2_1);
            double coord_x = (orient_flange(1,0)*std::cos(theta_1_2)-orient_flange(0,0)*std::sin(theta_1_2)) * std::sin(theta_5_2_1);
            theta_6_2_1 = std::atan2(coord_y, coord_x);
        }

        // Step 4: Calculate theta_3.  --> Two solutions for each pair of theta_1 and theta_5.
        // Firstly, we need to calculate 'theta_234 = theta_2-theta_3+theta_4'.
        Eigen::Matrix4d g_234_2_1 = xar::invTransf(xar::expMap(twists_[0], theta_1_2)) * pose_flange *
                xar::invTransf(xar::expMap(twists_[4], theta_5_2_1)*xar::expMap(twists_[5], theta_6_2_1)*pose_flange_home_);
        xar::ExpCoord rot_234_1 = xar::ExpCoord(Eigen::Matrix3d(g_234_2_1.block(0,0,3,3)));
        double theta_234_2_1 = rot_234_1.axis().dot(twists_[1].omg()) * rot_234_1.angle();

        // theta_234 is the composite effect of joint 2, 3, and 4, so it might be out of [-pi, pi].
        double theta_234_refer = joint_angle_ref[1] - joint_angle_ref[2] + joint_angle_ref[4];
        double dist_theta_234 = std::abs(theta_234_2_1-theta_234_refer);
        int num_round = 0;
        for (int idx_round = -2; idx_round <= 2; ++idx_round)
        {
            double dist_theta_234_round = std::abs((theta_234_2_1+double(idx_round)*(2.0*xar::PI)) - theta_234_refer);
            if (dist_theta_234_round < dist_theta_234)
            {
                dist_theta_234 = dist_theta_234_round;
                num_round = idx_round;
            }
        }
        theta_234_2_1 = theta_234_2_1 + double(num_round)*(2.0*xar::PI);

        double temp_m = -(g_234_2_1(0,3) - (len_1+len_3+len_5)*std::sin(theta_234_2_1));
        double temp_n = g_234_2_1(2,3) + (len_1+len_3+len_5)*std::cos(theta_234_2_1) - len_1;
        value_temp = (std::pow(temp_m,2) + std::pow(temp_n,2) - (std::pow(len_3,2) + std::pow(len_5,2))) / (2*len_3*len_5);
        if (std::abs(value_temp) < 1+EPS)
        {
            double theta_3_2_1_1 = xar::acos(value_temp);
            double theta_3_2_1_2 = -xar::acos(value_temp);

            // Step 5: Calculate theta_2.
            double coord_y = (len_5*std::cos(theta_3_2_1_1)+len_3)*temp_m + len_5*std::sin(theta_3_2_1_1)*temp_n;
            double coord_x = (len_5*std::cos(theta_3_2_1_1)+len_3)*temp_n - len_5*std::sin(theta_3_2_1_1)*temp_m;
            double theta_2_2_1_1 = std::atan2(coord_y, coord_x);
            coord_y = (len_5*std::cos(theta_3_2_1_2)+len_3)*temp_m + len_5*std::sin(theta_3_2_1_2)*temp_n;
            coord_x = (len_5*std::cos(theta_3_2_1_2)+len_3)*temp_n - len_5*std::sin(theta_3_2_1_2)*temp_m;
            double theta_2_2_1_2 = std::atan2(coord_y, coord_x);

            // Step 6: Calculate theta_4.
            double theta_4_2_1_1 = theta_234_2_1 - theta_2_2_1_1 + theta_3_2_1_1;
            double theta_4_2_1_2 = theta_234_2_1 - theta_2_2_1_2 + theta_3_2_1_2;

            // solution = [theta_1_2; theta_2_2_1_1; theta_3_2_1_1; theta_4_2_1_1; theta_5_2_1; theta_6_2_1];
            Eigen::VectorXd solution(6);
            solution(0) = theta_1_2;
            solution(1) = theta_2_2_1_1;
            solution(2) = theta_3_2_1_1;
            solution(3) = theta_4_2_1_1;//    v1 = vo.cross(v2);
            //    v3 = v1.cross(v2);
            solution(4) = theta_5_2_1;
            solution(5) = theta_6_2_1;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;

            // solution = [theta_1_2; theta_2_2_1_2; theta_3_2_1_2; theta_4_2_1_2; theta_5_2_1; theta_6_2_1];
            solution(0) = theta_1_2;
            solution(1) = theta_2_2_1_2;
            solution(2) = theta_3_2_1_2;
            solution(3) = theta_4_2_1_2;
            solution(4) = theta_5_2_1;
            solution(5) = theta_6_2_1;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;
        }

        double theta_6_2_2;
        if (std::abs(sin(theta_5_2_2)) < EPS)
        {
            theta_6_2_2 = joint_angle_ref(5);
        }
        else
        {
            double coord_y = (orient_flange(0,1)*std::sin(theta_1_2)-orient_flange(1,1)*std::cos(theta_1_2))*std::sin(theta_5_2_2);
            double coord_x = (orient_flange(1,0)*std::cos(theta_1_2)-orient_flange(0,0)*std::sin(theta_1_2))*std::sin(theta_5_2_2);
            theta_6_2_2 = std::atan2(coord_y, coord_x);
        }

        Eigen::Matrix4d g_234_2_2 = xar::invTransf(xar::expMap(twists_[0], theta_1_2)) * pose_flange *
                xar::invTransf(xar::expMap(twists_[4], theta_5_2_2)*xar::expMap(twists_[5], theta_6_2_2)*pose_flange_home_);
        xar::ExpCoord rot_234_2 = xar::ExpCoord(Eigen::Matrix3d(g_234_2_2.block(0,0,3,3)));
        double theta_234_2_2 = rot_234_2.axis().dot(twists_[1].omg()) * rot_234_2.angle();
        dist_theta_234 = std::abs(theta_234_2_2-theta_234_refer);
        num_round = 0;
        for (int idx_round = -2; idx_round <= 2; ++idx_round)
        {
            double dist_theta_234_round = std::abs((theta_234_2_2+double(idx_round)*(2.0*xar::PI)) - theta_234_refer);
            if (dist_theta_234_round < dist_theta_234)
            {
                dist_theta_234 = dist_theta_234_round;
                num_round = idx_round;
            }
        }
        theta_234_2_2 = theta_234_2_2 + double(num_round)*(2.0*xar::PI);

        temp_m = -(g_234_2_2(0,3) - (len_1+len_3+len_5)*sin(theta_234_2_2));
        temp_n = g_234_2_2(2,3) + (len_1+len_3+len_5)*cos(theta_234_2_2) - len_1;
        value_temp = (std::pow(temp_m,2) + std::pow(temp_n,2) - (std::pow(len_3,2) + std::pow(len_5,2))) / (2*len_3*len_5);
        if (std::abs(value_temp) < 1+EPS)
        {
            double theta_3_2_2_1 = xar::acos(value_temp);
            double theta_3_2_2_2 = -xar::acos(value_temp);

            // Step 5: Calculate theta_2.
            double coord_y = (len_5*std::cos(theta_3_2_2_1)+len_3)*temp_m + len_5*std::sin(theta_3_2_2_1)*temp_n;
            double coord_x = (len_5*std::cos(theta_3_2_2_1)+len_3)*temp_n - len_5*std::sin(theta_3_2_2_1)*temp_m;
            double theta_2_2_2_1 = std::atan2(coord_y, coord_x);
            coord_y = (len_5*std::cos(theta_3_2_2_2)+len_3)*temp_m + len_5*std::sin(theta_3_2_2_2)*temp_n;
            coord_x = (len_5*std::cos(theta_3_2_2_2)+len_3)*temp_n - len_5*std::sin(theta_3_2_2_2)*temp_m;
            double theta_2_2_2_2 = std::atan2(coord_y, coord_x);

            // Step 6: Calculate theta_4.
            double theta_4_2_2_1 = theta_234_2_2 - theta_2_2_2_1 + theta_3_2_2_1;
            double theta_4_2_2_2 = theta_234_2_2 - theta_2_2_2_2 + theta_3_2_2_2;

            // solution = [theta_1_2; theta_2_2_2_1; theta_3_2_2_1; theta_4_2_2_1; theta_5_2_2; theta_6_2_2];
            Eigen::VectorXd solution(6);
            solution(0) = theta_1_2;
            solution(1) = theta_2_2_2_1;
            solution(2) = theta_3_2_2_1;
            solution(3) = theta_4_2_2_1;
            solution(4) = theta_5_2_2;
            solution(5) = theta_6_2_2;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;

            // solution = [theta_1_2; theta_2_2_2_2; theta_3_2_2_2; theta_4_2_2_2; theta_5_2_2; theta_6_2_2];
            solution(0) = theta_1_2;
            solution(1) = theta_2_2_2_2;
            solution(2) = theta_3_2_2_2;
            solution(3) = theta_4_2_2_2;
            solution(4) = theta_5_2_2;
            solution(5) = theta_6_2_2;
            valid_solutions.block(0,valid_count,6,1) = solution;
            valid_count = valid_count + 1;
        }
    }

    double dist = 9999;  	// Choose the nearest solution.
    bool is_found = false;
    for (int idx_solution = 0; idx_solution < valid_count; ++idx_solution)
    {
        bool is_out_of_range = false;
        for (int idx_joint = 0; idx_joint < 6; ++idx_joint)
        {
            if (std::abs(valid_solutions(idx_joint, idx_solution)) > 174.0*xar::PI/180.0)
            {
                is_out_of_range = true;
                break;
            }
        }

        if (!is_out_of_range)
        {
            double dist_solution = (valid_solutions.block(0,idx_solution,6,1)-joint_angle_ref).norm();
            if (dist_solution < dist)
            {
                is_found = true;
                joint_angle = valid_solutions.block(0,idx_solution,6,1);
                dist = dist_solution;
            }
        }
    }

    return is_found;
}
//
int AuboRobot::getRobotState(aubo_robot_namespace::RobotState & state)
{
    return robotService.robotServiceGetRobotCurrentState(state);
}

int AuboRobot::setWorkMode(aubo_robot_namespace::RobotWorkMode mode)
{
    return robotService.robotServiceSetRobotWorkMode(mode);
}

int AuboRobot::getWorkMode(aubo_robot_namespace::RobotWorkMode & mode)
{
    return robotService.robotServiceGetRobotWorkMode(mode);
}

//
int AuboRobot::getDeviceInfo(aubo_robot_namespace::RobotDevInfo & device_info)
{
    return robotService.robotServiceGetRobotDevInfoService(device_info);
}

// Set the properties of IO pins on the flange.
int AuboRobot::setIoProperty()
{
    return 0;
}

int getIoProperty()
{
    return 0;
}

// Register callback functions.
int AuboRobot::registerCallbackFunction(void * ptr_function, void * arg, CallbackType type)
{
    switch (type)
    {
        case CallbackRoadPoint:
            return robotService.robotServiceRegisterRealTimeRoadPointCallback((RealTimeRoadPointCallback)ptr_function, arg);
        case CallbackJointStatus:
            return robotService.robotServiceRegisterRealTimeJointStatusCallback((RealTimeJointStatusCallback)ptr_function, arg);
        case CallbackEndSpeed:
            return robotService.robotServiceRegisterRealTimeEndSpeedCallback((RealTimeEndSpeedCallback)ptr_function, arg);
        case CallbackRobotEvent:
            return robotService.robotServiceRegisterRobotEventInfoCallback((RobotEventCallback)ptr_function, arg);
        default:
            return 1;
    }
}

int AuboRobot::setCallbackStatus(bool enable, CallbackType type)
{
    switch (type)
    {
        case CallbackRoadPoint:
            return robotService.robotServiceSetRealTimeRoadPointPush(enable);
        case CallbackJointStatus:
            return robotService.robotServiceSetRealTimeJointStatusPush(enable);
        case CallbackEndSpeed:
            return robotService.robotServiceSetRealTimeEndSpeedPush(enable);
        case CallbackRobotEvent:
            return 1;
        default:
            return 1;
    }
}

void AuboRobot::setConfiguration()
{
    //i5
//    double len_1 = 98.5 / 1000.0;
//    double len_2 = 140.5 / 1000.0;
//    double len_3 = 408.0 / 1000.0;
//    double len_4 = 121.5 / 1000.0;
//    double len_5 = 376.0 / 1000.0;
//    double len_6 = 102.5 / 1000.0;
//    double len_7 = 102.5 / 1000.0;
//    double len_8 = 94.0 / 1000.0;

    //I10
    double len_1 = 163.0 / 1000.0;
    double len_2 = 197.0 / 1000.0;
    double len_3 = 647.0 / 1000.0;
    double len_4 = 123.5 / 1000.0;
    double len_5 = 600.5 / 1000.0;
    double len_6 = 127.8 / 1000.0;
    double len_7 = 102.5 / 1000.0;
    double len_8 = 94.0 / 1000.0;
    len_params_ = std::vector<double>(8);
    len_params_[0] = len_1;
    len_params_[1] = len_2;
    len_params_[2] = len_3;
    len_params_[3] = len_4;
    len_params_[4] = len_5;
    len_params_[5] = len_6;
    len_params_[6] = len_7;
    len_params_[7] = len_8;

    twists_ = std::vector<xar::Twist>(6);
    twists_[0] = xar::Twist(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0));
    twists_[1] = xar::Twist(Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0.0, 0.0, len_1));
    twists_[2] = xar::Twist(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0.0, 0.0, len_1+len_3));
    twists_[3] = xar::Twist(Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0.0, 0.0, len_1+len_3+len_5));
    twists_[4] = xar::Twist(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.0, -(len_2-len_4+len_6), 0.0));
    twists_[5] = xar::Twist(Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0.0, 0.0, len_1+len_3+len_5+len_7));

    pose_flange_home_ = Eigen::Matrix4d::Identity();
    pose_flange_home_(1,3) = -(len_2 - len_4 + len_6 + len_8);
    pose_flange_home_(2,3) = len_1 + len_3 + len_5 + len_7;
    pose_flange_home_.block(0, 0, 3, 3) = xar::expMap(Eigen::Vector3d(1, 0, 0), xar::PI/2.0);
}

bool AuboRobot::checkSolutionValid(const Eigen::Matrix4d & pose_flange_ref, const Eigen::VectorXd & joint_angles, double eps_pos, double eps_orient)
{
    Eigen::Matrix4d pose_flange = xar::prodOfExp(twists_, joint_angles, pose_flange_home_);
    Eigen::Vector3d error_position = pose_flange.block(0,3,3,1) - pose_flange_ref.block(0,3,3,1);
    xar::ExpCoord error_orient = xar::ExpCoord(Eigen::Matrix3d(pose_flange.block(0,0,3,3) * pose_flange_ref.block(0,0,3,3).transpose()));

    if ((error_position.norm() <= eps_pos) && (error_orient.angle() <= eps_orient))
        return true;
    else
        return false;
}

void AuboRobot::getJacobian(const Eigen::VectorXd & joint_angle, Eigen::MatrixXd & mat_jacob)
{
    Eigen::Vector3d pos_flange = xar::prodOfExp(twists_, joint_angle, Eigen::Vector3d(pose_flange_home_.block(0,3,3,1)));
    mat_jacob = xar::calJacobian(twists_, joint_angle, pos_flange);
}

//int AuboRobot::Move_Rot(double ro[3],double ang)
//{
//    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool refer;
//    int result;
//    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
//    toolInEndDesc.toolInEndPosition.x=0.0;
//    toolInEndDesc.toolInEndPosition.y=0.0;
//    toolInEndDesc.toolInEndPosition.z=0.0;
//    toolInEndDesc.toolInEndOrientation.w=1.0;
//    toolInEndDesc.toolInEndOrientation.x=0.0;
//    toolInEndDesc.toolInEndOrientation.y=0.0;
//    toolInEndDesc.toolInEndOrientation.z=0.0;
//    refer.coordType = aubo_robot_namespace::coordinate_refer::BaseCoordinate;
//    refer.coordType = aubo_robot_namespace::coordinate_refer::EndCoordinate;
//    refer.toolDesc = toolInEndDesc;
//    Eigen::Matrix4d T;
//    getFlangePose(T);
//    Eigen::MatrixXd roo;
//    roo = Eigen::MatrixXd::Zero(3,1);
//    roo(0,0) = ro[0];
//    roo(1,0) = ro[1];
//    roo(2,0) = ro[2];
//    roo = T.block(0,0,3,3)*roo;
//    double rota[3];
//    rota[0] = roo(0,0);
//    rota[1] = roo(1,0);
//    rota[2] = roo(2,0);
//    result = robotService.robotServiceRotateMove(refer,rota,ang,true);
//    return result;
//}
// a -> x0 y1 z2; b->StepDistance
//int AuboRobot::Move_Line(int a, float b)
//{
//    Eigen::Matrix4d flange_pose;
//    Eigen::Matrix4d flange_pose_copy;
//    Eigen::VectorXd joint_angle_refer,joint_angel;
//    getFlangePose(flange_pose);
//    flange_pose_copy = flange_pose;
//    Eigen::MatrixXd T;
//    T = Eigen::MatrixXd::Zero(3,1);
//    T(a,0)= b ;
//    flange_pose_copy.block(0,3,3,1)=flange_pose_copy.block(0,3,3,1)+flange_pose_copy.block(0,0,3,3)*T;
//    getJointAngle(joint_angle_refer);
//    int r =0;
//    r = inverseKinematics(joint_angle_refer,flange_pose_copy,joint_angel);
//    if (r == 0)
//    {
//        return 1;
//    }
//    return moveline(joint_angel,true);
//}

//int AuboRobot::Move_Line_Press(int a,float b)
//{
//    Eigen::Matrix4d flange_pose;
//    Eigen::Matrix4d flange_pose_copy;
//    Eigen::VectorXd joint_angle_refer,joint_angel;
//    getFlangePose(flange_pose);
//    flange_pose_copy = flange_pose;
//    Eigen::MatrixXd T;
//    T = Eigen::MatrixXd::Zero(3,1);
//    int r = 0;
//    int limit = 0;
//    T(a,0)= b;
//    while (1)
//    {
//        flange_pose_copy.block(0,3,3,1)=flange_pose_copy.block(0,3,3,1)+flange_pose_copy.block(0,0,3,3)*T;
//        getJointAngle(joint_angle_refer);
//        r = inverseKinematics(joint_angle_refer,flange_pose_copy,joint_angel) ;
//        if (r == 1)
//        {
//            b = b+0.01;
//            T(a,0)= b;
//            if (limit = 0)
//            {
//                limit = 1;
//            }
//        }
//        else
//        {
//            b = b-0.01;
//            T(a,0)= b;
//            if (limit = 1)
//            {
//                break;
//            }
//        }
//    }
//    return moveline(joint_angel,true);
//}
//int AuboRobot::Jointone_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(0,0)=joint_angle(0,0)+ang;
//    return move(joint_angle,true);
//}
//int AuboRobot::Jointtwo_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(1,0)=joint_angle(1,0)+ang;
//    return move(joint_angle,true);
//}
//int AuboRobot::Jointthree_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(2,0)=joint_angle(2,0)+ang;
//    return move(joint_angle,true);
//}
//int AuboRobot::Jointfour_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(3,0)=joint_angle(3,0)+ang;
//    return move(joint_angle,true);
//}
//int AuboRobot::Jointfive_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(4,0)=joint_angle(4,0)+ang;
//    return move(joint_angle,true);
//}
//int AuboRobot::Jointsix_move(float ang)
//{
//    Eigen::VectorXd joint_angle;
//    joint_angle=Eigen::MatrixXd::Zero(6,1);
//    getJointAngle(joint_angle);//get current angle
//    joint_angle(5,0)=joint_angle(5,0)+ang;
//    return move(joint_angle,true);
//}

//0316 teach move and teach stop

int AuboRobot::teach_move(aubo_robot_namespace::teach_mode mode,bool direction)
{
   int res=0;
   aubo_robot_namespace::CoordCalibrateByJointAngleAndTool refer;
   refer.coordType = aubo_robot_namespace::coordinate_refer::EndCoordinate;
   aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
   toolInEndDesc.toolInEndPosition.x=0.0;
   toolInEndDesc.toolInEndPosition.y=0.0;
   toolInEndDesc.toolInEndPosition.z=0.0;
   toolInEndDesc.toolInEndOrientation.w=1.0;
   toolInEndDesc.toolInEndOrientation.x=0.0;
   toolInEndDesc.toolInEndOrientation.y=0.0;
   toolInEndDesc.toolInEndOrientation.z=0.0;
   refer.toolDesc = toolInEndDesc;
   robotService.robotServiceSetTeachCoordinateSystem(refer);
   res = robotService.robotServiceTeachStart(mode,direction);
   if (res == 0)
       return true;
   else
       return false;
}

int AuboRobot::teach_stop()
{
   int res=0;
   res = robotService.robotServiceTeachStop();
   if (res == 0)
       return true;
   else
       return false;
}

