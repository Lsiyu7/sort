#ifndef ANTHROPARM_H
#define ANTHROPARM_H

#include <vector>
#include "twist.h"

namespace xar {

typedef Eigen::VectorXd (*FuncJointMotionDeisgn)(const std::vector<double> & angle_primitives, double time);
typedef Twist (*FuncCartesianMotionDeisgn)(const Eigen::Matrix4d & pose_wrist_to_tool,
        const Eigen::Matrix4d & pose_tool_init, double time);

class AnthropArm
{
public:
    AnthropArm();
    ~AnthropArm();

    void setLength(double len_upperarm, double len_forarm);
    void setTwists(const std::vector<Twist> & twists);
    void setPoseElbowHome(const Eigen::Matrix4d & pose_elbow_home);
    void setPoseWristHome(const Eigen::Matrix4d & pose_wrist_home);
    void setPoseWristToTool(const Eigen::Matrix4d & pose_wrist_to_tool);

    void getLength(double & len_upperarm, double & len_forarm);
    void getTwists(std::vector<Twist> & twists);
    void getPoseElbowHome(Eigen::Matrix4d & pose_elbow_home);
    void getPoseWristHome(Eigen::Matrix4d & pose_wrist_home);
    void getPoseWristToTool(Eigen::Matrix4d & pose_wrist_to_tool);

    void armConfig2JointConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle, Eigen::VectorXd & joint_angles);
    void jointConfig2ArmConfig(const Eigen::VectorXd & joint_angles, Eigen::Matrix4d & pose_wrist, double & arm_angle);

    void setInitialConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle);
    void setInitialConfig(const Eigen::VectorXd & joint_angles);

    void getInitialConfig(Eigen::Matrix4d & pose_wrist, double & arm_angle);
    void getInitialConfig(Eigen::VectorXd & joint_angles);

    // Set the final configuration. This is ONLY for joint motion planning.
    void setFinalConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle);
    void setFinalConfig(const Eigen::VectorXd & joint_angles);

    void getFinalConfig(Eigen::Matrix4d & pose_wrist, double & arm_angle);
    void getFinalConfig(Eigen::VectorXd & joint_angles);

    void setTimeTotal(double time_total);
    void setTimeStep(double time_step);

    void getTimeTotal(double & time_total);
    void getTimeStep(double & time_step);

    void computeMotionPrimitives(std::vector<Twist> & twist_primitives, std::vector<double> & angle_primitives);
    void computeConsistentSolution(const Eigen::VectorXd & joint_angles, const Twist & vel_tool, Eigen::VectorXd &joint_velocities);

    void plan(FuncJointMotionDeisgn func_motion_design);
    void plan(FuncCartesianMotionDeisgn func_motion_design);

private:
    double time_total_;
    double time_step_;
    double len_upperarm_;
    double len_forearm_;
    std::vector<Twist> twists_;
    Eigen::Vector3d vect_ref_;
    Eigen::Matrix4d pose_elbow_home_;
    Eigen::Matrix4d pose_wrist_home_;
    Eigen::Matrix4d pose_wrist_to_tool_;
    Eigen::VectorXd joint_angles_init_;
    Eigen::Matrix4d pose_elbow_init_;
    Eigen::Matrix4d pose_wrist_init_;
    Eigen::VectorXd joint_angles_final_;
    Eigen::Matrix4d pose_elbow_final_;
    Eigen::Matrix4d pose_wrist_final_;
};

Eigen::Vector3d invSphericalJoint(const std::vector<Eigen::Vector3d> & triple_axes, const Eigen::Matrix3d & rotation_matrix);

}

#endif // ANTHROPARM_H
