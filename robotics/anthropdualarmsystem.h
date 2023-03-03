#ifndef ANTHROPDUALARMSYSTEM_H
#define ANTHROPDUALARMSYSTEM_H

#include "anthroparm.h"

namespace xar {

enum ArmID {LeftArm, RightArm};

class AnthropDualArmSystem
{
public:
    AnthropDualArmSystem();
    ~AnthropDualArmSystem();

    void setLength(double len_upperarm, double len_forarm, ArmID arm_id);
    void setTwists(const std::vector<Twist> & twists, ArmID arm_id);
    void setPoseElbowHome(const Eigen::Matrix4d & pose_elbow_home, ArmID arm_id);
    void setPoseWristHome(const Eigen::Matrix4d & pose_wrist_home, ArmID arm_id);
    void setPoseWristToTool(const Eigen::Matrix4d & pose_wrist_to_tool, ArmID arm_id);
    void setPoseToolToObject(const Eigen::Matrix4d & pose_tool_to_object, ArmID arm_id);

    void armConfig2JointConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle, Eigen::VectorXd & joint_angles, ArmID arm_id);
    void jointConfig2ArmConfig(const Eigen::VectorXd & joint_angles, Eigen::Matrix4d & pose_wrist, double & arm_angle, ArmID arm_id);

    void setInitialConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle, ArmID arm_id);
    void setInitialConfig(const Eigen::VectorXd & joint_angles, ArmID arm_id);

    // Set the final configuration. This is ONLY for joint motion planning.
    void setFinalConfig(const Eigen::Matrix4d & pose_wrist, double arm_angle, ArmID arm_id);
    void setFinalConfig(const Eigen::VectorXd & joint_angles, ArmID arm_id);

    void computeMotionPrimitives(std::vector<Twist> & twist_primitives_left, std::vector<double> & angle_primitives_left,
            std::vector<Twist> & twist_primitives_right, std::vector<double> & angle_primitives_right);
    void computeConsistentSolution(const Eigen::VectorXd & joint_angles_left, const Eigen::VectorXd & joint_angles_right,
            const Twist & vel_object, Eigen::VectorXd & joint_velocities_left, Eigen::VectorXd & joint_velocities_right);

    void plan(FuncJointMotionDeisgn func_motion_design);
    void plan(FuncCartesianMotionDeisgn func_motion_design);

private:
    AnthropArm arm_left_;
    AnthropArm arm_right_;
    Eigen::Matrix4d pose_tool_to_object_left_;
    Eigen::Matrix4d pose_tool_to_object_right_;
    double time_total_;
    double time_step_;
};

}

#endif // ANTHROPDUALARMSYSTEM_H
