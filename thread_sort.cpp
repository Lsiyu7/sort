#include "thread_sort.h"


thread_sort::thread_sort()
{
    //旋转矩阵
//    rotateMat<<-0.0199944,0.999797,0.00228692,
//            -0.991839,-0.0201232,0.125894,
//            0.125915,0.000248965,0.992041;

    sort = new SortFruit();
    action = " ";
    memset(sort_info,0,sizeof(sort_info));
}

thread_sort::~thread_sort()
{

}

int thread_sort::connect_arm()
{
    if(sort->connect("192.168.0.102",8899))
    {
      return 0;
    }
    return 1;
}

int thread_sort::connect_status()
{
    if (!sort->isConnected())
    {
        return 0;
    }
    return 1;
}

void thread_sort::fast_stop()
{
    sort->ClearTrajectory();
    sort->shutdown();
}

void thread_sort::get_info(float info[9])
{
    sort_info[0] = info[0];
    sort_info[1] = info[1];
    sort_info[2] = info[2];
    sort_info[3] = info[3];
    sort_info[4] = info[4];
    sort_info[5] = info[5];
    sort_info[6] = info[6];
    sort_info[7] = info[7];
    sort_info[8] = info[8];
}

void thread_sort::get_action(QString act)
{
    action = act;
}

void thread_sort::Delay(int msec)
{
    QEventLoop loop;
    QTimer::singleShot(msec,&loop,SLOT(quit()));
    loop.exec();
}

void thread_sort::run()
{

    if (action == "pick_approach")
    {
        if (sort_info[0] == 0)
        {
            emit fail("no_image\n");
            return;
        }
        sort->stop_move = false;
        sort->GetPoint(sort_info[0],sort_info[1],sort_info[2]);
        sort->Pick_ComputeSample(originPoint,quaternion,sort_info);//平移矩阵，旋转矩阵
        if (!sort->Pick_ComputeIK())
        {
            emit fail("no_inverse\n");
            return;
        }
        int r = sort->Pick_Approach();
        if (r == -2)
        {
            emit fail("joint_limit\n");
            return;
        }
        emit tool_control("tighten_bolt");
        Delay(4000);
        r = sort->Pick_Approach_2();
        if (r == -2)
        {
            emit fail("joint_limit\n");
            return;
        }
        Delay(15000);
        emit tool_control("down_sleeve");
        Delay(10000);
        emit tool_control("open_gripper");
        Delay(10000);
        memset(sort_info,0,sizeof(sort_info));
        emit fail("success=1\n");
    }
    else if (action == "pick_retreat")
    {
        if(sort->HaveTrajectory() == false)
        {
            emit fail("no_trajectory\n");
            return;
        }
        int r = sort->Pick_Retreat();
        if (r == -2)
        {
            emit fail("joint_limit\n");
            return;
        }
        sort->ClearTrajectory();
        emit fail("success=2\n");
    }
    else if (action == "place_approach")
    {
        if (sort_info[0] == 0)
        {
            emit fail("no_image\n");
            return;
        }
        sort->stop_move = false;
        sort->GetPoint(sort_info[0],sort_info[1],sort_info[2]);
        sort->Place_ComputeSample(originPoint,quaternion,sort_info);//平移矩阵，旋转矩阵
        if (!sort->Place_ComputeIK())
        {
            emit fail("no_inverse\n");
            return;
        }
        int r = sort->Place_Approach();
        if (r == -2)
        {
            emit fail("joint_limit\n");
            return;
        }
        memset(sort_info,0,sizeof(sort_info));
        emit tool_control("close_gripper");
        Delay(10000);
        emit tool_control("up_sleeve");
        Delay(10000);
        //
        emit tool_control("loost_bolt");
        Delay(1000);
        emit tool_control("close_gripper");
        Delay(1000);
        emit tool_control("close_gripper");
        Delay(1000);
        emit tool_control("close_gripper");
        Delay(1000);
        emit tool_control("close_gripper");
        Delay(10000);
        emit fail("success=3\n");
    }
    else if (action == "place_retreat")
    {
        if(sort->HaveTrajectory() == false)
        {
            emit fail("no_trajectory\n");
            return;
        }
        int r = sort->Place_Retreat();
        if (r == -2)
        {
            emit fail("joint_limit\n");
            return;
        }
        sort->ClearTrajectory();
        emit fail("success=4\n");
    }
    else if (action == "tech_stop")
    {
        int result = sort->teach_stop();
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "x_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_X,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "x_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_X,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "y_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_Y,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "y_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_Y,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "z_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_Z,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "z_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::MOV_Z,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "rx_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_X,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "rx_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_X,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "ry_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_Y,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "ry_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_Y,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "rz_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_Z,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "rz_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::ROT_Z,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j1_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT1,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j1_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT1,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j2_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT2,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j2_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT2,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j3_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT3,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j3_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT3,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j4_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT4,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j4_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT4,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j5_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT5,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j5_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT5,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j6_plus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT6,true);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "j6_minus")
    {
        int result = sort->teach_move(aubo_robot_namespace::teach_mode::JOINT6,false);
        if (result != 1)
        {
            emit fail("joint_limit\n");
            return;
        }
    }
    else if (action == "fast_stop")
    {
        sort->fastStop();
    }
    else if (action == "start_up")
    {
        sort->startup();
    }
    else if (action == "shut_down")
    {
        sort->shutdown();
    }
    else if (action == "return")
    {
        sort->ClearTrajectory();
        Eigen::VectorXd joint_angle;
        joint_angle=Eigen::MatrixXd::Zero(6,1);
        sort->getJointAngle(joint_angle);
        joint_angle(0,0)=xar::deg2rad(84.848);
        sort->move(joint_angle,true);

        Eigen::VectorXd initialangel(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        sort->move(initialangel,true);

    }
    else if (action == "pick_line1")
    {

        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(-4.991),xar::deg2rad(46.860),xar::deg2rad(-83.122),
                xar::deg2rad(49.746),xar::deg2rad(-94.410),xar::deg2rad(1.073);
        waypoint3<<xar::deg2rad(-7.458),xar::deg2rad(28.230),xar::deg2rad(-120.549),
                xar::deg2rad(30.953),xar::deg2rad(-96.898),xar::deg2rad(1.034);
        waypointpickline<<xar::deg2rad(-7.458),xar::deg2rad(30.248),xar::deg2rad(-120.726),
                xar::deg2rad(28.759),xar::deg2rad(-96.902),xar::deg2rad(1.032);

        emit tool_control("down_sleeve");
        emit tool_control("open_gripper");
        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("close_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        emit tool_control("up_sleeve");
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=5\n");
    }
    else if (action == "pick_line2")
    {
        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(6.586),xar::deg2rad(44.357),xar::deg2rad(-88.789),
                xar::deg2rad(47.817),xar::deg2rad(-83.116),xar::deg2rad(1.020);
        waypoint3<<xar::deg2rad(8.667),xar::deg2rad(32.258),xar::deg2rad(-113.296),
                xar::deg2rad(35.410),xar::deg2rad(-81.049),xar::deg2rad(0.966);
        waypointpickline<<xar::deg2rad(8.667),xar::deg2rad(33.964),xar::deg2rad(-113.438),
                xar::deg2rad(33.564),xar::deg2rad(-81.051),xar::deg2rad(0.964);

        emit tool_control("down_sleeve");
        emit tool_control("open_gripper");
        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("close_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        emit tool_control("up_sleeve");
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=7\n");
    }
    else if (action == "pick_line3")
    {
        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(19.255),xar::deg2rad(51.789),xar::deg2rad(-73.944),
                xar::deg2rad(55.937),xar::deg2rad(-81.114),xar::deg2rad(0.998);
        waypoint3<<xar::deg2rad(24.636),xar::deg2rad(39.276),xar::deg2rad(-99.458),
                xar::deg2rad(42.984),xar::deg2rad(-75.752),xar::deg2rad(0.820);
        waypointpickline<<xar::deg2rad(24.635),xar::deg2rad(40.814),xar::deg2rad(-99.588),
                xar::deg2rad(41.288),xar::deg2rad(-75.755),xar::deg2rad(0.818);
        emit tool_control("down_sleeve");
        emit tool_control("open_gripper");
        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("close_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        emit tool_control("up_sleeve");
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=9\n");
    }

    else if (action == "place_line1")
    {
        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);

        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(-4.991),xar::deg2rad(46.860),xar::deg2rad(-83.122),
                xar::deg2rad(49.746),xar::deg2rad(-94.410),xar::deg2rad(1.073);
        waypoint3<<xar::deg2rad(-7.458),xar::deg2rad(28.230),xar::deg2rad(-120.549),
                xar::deg2rad(30.953),xar::deg2rad(-96.898),xar::deg2rad(1.034);
        waypointpickline<<xar::deg2rad(-7.458),xar::deg2rad(30.248),xar::deg2rad(-120.726),
                xar::deg2rad(28.759),xar::deg2rad(-96.902),xar::deg2rad(1.032);

        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        emit tool_control("down_sleeve");
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("open_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=6\n");
    }
    else if (action == "place_line2")
    {
        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(6.586),xar::deg2rad(44.357),xar::deg2rad(-88.789),
                xar::deg2rad(47.817),xar::deg2rad(-83.116),xar::deg2rad(1.020);
        waypoint3<<xar::deg2rad(8.667),xar::deg2rad(32.258),xar::deg2rad(-113.296),
                xar::deg2rad(35.410),xar::deg2rad(-81.049),xar::deg2rad(0.966);
        waypointpickline<<xar::deg2rad(8.667),xar::deg2rad(33.964),xar::deg2rad(-113.438),
                xar::deg2rad(33.564),xar::deg2rad(-81.051),xar::deg2rad(0.964);

        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        emit tool_control("down_sleeve");
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("open_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=8\n");
    }
    else if (action == "place_line3")
    {
        Eigen::VectorXd initialangel(6,1);
        Eigen::VectorXd waypointstart(6,1);
        Eigen::VectorXd waypoint2(6,1);
        Eigen::VectorXd waypoint3(6,1);
        Eigen::VectorXd waypointpickline(6,1);
        initialangel<<xar::deg2rad(84.848),xar::deg2rad(-41.730),xar::deg2rad(-132.000),
                xar::deg2rad(-96.636),xar::deg2rad(-82.066),xar::deg2rad(-89.00);
        waypointstart<<xar::deg2rad(-8.533),xar::deg2rad(34.759),xar::deg2rad(-96.571),
                xar::deg2rad(-9.651),xar::deg2rad(-9.176),xar::deg2rad(-28.506);
        waypoint2<<xar::deg2rad(19.255),xar::deg2rad(51.789),xar::deg2rad(-73.944),
                xar::deg2rad(55.937),xar::deg2rad(-81.114),xar::deg2rad(0.998);
        waypoint3<<xar::deg2rad(24.636),xar::deg2rad(39.276),xar::deg2rad(-99.458),
                xar::deg2rad(42.984),xar::deg2rad(-75.752),xar::deg2rad(0.820);
        waypointpickline<<xar::deg2rad(24.635),xar::deg2rad(40.814),xar::deg2rad(-99.588),
                xar::deg2rad(41.288),xar::deg2rad(-75.755),xar::deg2rad(0.818);

        sort->move(initialangel,true);
        sort->move(waypointstart,true);
        emit tool_control("down_sleeve");
        sort->move(waypoint2,true);
        sort->move(waypoint3,true);
        sort->move(waypointpickline,true);
        emit tool_control("open_gripper");
        Delay(13000);
        sort->move(waypoint3,true);
        sort->move(waypoint2,true);
        sort->move(waypointstart,true);
        sort->move(initialangel,true);
        emit fail("success=10\n");
    }
}

void thread_sort::stop()   //stop the thread
{
    pthread_mutex_lock(&stop_move_mutex);//变量锁
    sort->stop_move = true;
    pthread_mutex_unlock(&stop_move_mutex);
    sort->stop();
    sort->ClearTrajectory();
}


