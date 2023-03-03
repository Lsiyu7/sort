#include "sortfruit.h"

SortFruit::SortFruit()
{
    stop_move = false;
}
Eigen::Quaterniond SortFruit::euler2Quaternion(const double roll, const double pitch, const double yaw)\
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    return q;
}
Eigen::Vector3d SortFruit::Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
     return  euler;
}

Eigen::Matrix3d SortFruit::euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d R = q.matrix();
    return R;
}
Eigen::Vector3d SortFruit::RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
    return euler;
}

Eigen::Matrix3d SortFruit::Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
   Eigen::Quaterniond q;
   q.x() = x;
   q.y() = y;
   q.z() = z;
   q.w() = w;
   //q.toRotationMatrix()
   Eigen::Matrix3d R = q.normalized().toRotationMatrix();
   return  R;
}
Eigen::Quaterniond SortFruit::rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    return q;
}

void SortFruit::GetPoint(double x,double y,double z)
{
    Eigen::MatrixXd wayPoint=Eigen::MatrixXd::Zero(3,1);
    wayPoint(0,0) =x;
    wayPoint(1,0) =y;
    wayPoint(2,0) =z;
    m_wayPoint = wayPoint;
}

void SortFruit::ClearTrajectory()
{
    m_strokeIK = {};
}

bool SortFruit::HaveTrajectory()
{
    if (m_strokeIK[0].toArray()[0].toArray().size() == 0)
    {
        return false;
    }
    return true;
}

//camera to base
Eigen::MatrixXd SortFruit::Pick_ComputeSample(double initPos[3],Eigen::Quaterniond quaternion,float info[9])
{
    Eigen::MatrixXd T;
    //平移矩阵
    T = Eigen::MatrixXd::Ones(3,m_wayPoint.cols());
    T.row(0)=T.row(0)*initPos[0]; //x
    T.row(1)=T.row(1)*initPos[1]; //y
    T.row(2)=T.row(2)*initPos[2]; //z

    Eigen::MatrixXd p1;
    Eigen::MatrixXd p2;
    p1 = Eigen::MatrixXd::Zero(3,1);
    p1(0,0)=info[3];
    p1(1,0)=info[4];
    p1(2,0)=info[5];
    p2 = Eigen::MatrixXd::Zero(3,1);
    p2(0,0)=info[6];
    p2(1,0)=info[7];
    p2(2,0)=info[8];
    qDebug()<< p1(0,0) << p1(1,0) << p1(2,0) << "\n";
    qDebug()<< m_wayPoint(0,0) << m_wayPoint(1,0) << m_wayPoint(2,0) << "\n";
    qDebug()<< p2(0,0) << p2(1,0) << p2(2,0) << "\n";

    Eigen::Matrix3d  rotateMat = quaternion.toRotationMatrix();
    p1 = rotateMat*p1+T;
    p2 = rotateMat*p2+T;
    m_wayPoint = rotateMat*m_wayPoint+T;
    Eigen::Matrix4d T2;
    Eigen::Matrix3d T3;
    Eigen::MatrixXd T4;
    getFlangePose(T2);
    T3 = T2.block(0,0,3,3);
    T4 = T2.block(0,3,3,1);
    p1 = T3*p1+T4;
    p2 = T3*p2+T4;
    m_wayPoint = T3*m_wayPoint+T4;
    qDebug()<< p1(0,0) << p1(1,0) << p1(2,0) << "\n";
    qDebug()<< m_wayPoint(0,0) << m_wayPoint(1,0) << m_wayPoint(2,0) << "\n";
    qDebug()<< p2(0,0) << p2(1,0) << p2(2,0) << "\n";

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Vector3d vo(0.0,0.0,1.0);

    v1(0) = p1(0,0)-p2(0,0);
    v1(1) = p1(1,0)-p2(1,0);
    v1(2) = p1(2,0)-p2(2,0);
    v3 = v1.cross(vo);
    v2 = v3.cross(v1);
    v1.normalize();
    v2.normalize();
    v3.normalize();
    TB.block(0,0,3,1) = v1;
    TB.block(0,1,3,1) = v2;
    TB.block(0,2,3,1) = v3;
    return m_wayPoint;
}

bool SortFruit::Pick_ComputeIK()
{
    QJsonArray IKArray,IKangel1,IKangel2,IKangel3,IKangel4,IKangel5,IKangel6;
    Eigen::Matrix4d s=Eigen::Matrix4d::Identity();
    Eigen::VectorXd Sangle1(6,1),Sangle2(6,1),Sangle3(6,1),Sangle4(6,1),Sangle5(6,1),endangel(6,1),endangel2(6,1);
    Eigen::Matrix4d s1=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d s2=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d s3=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d s4=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d s5=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP1=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP2=Eigen::Matrix4d::Identity();

    Eigen::MatrixXd InverseK = Eigen::MatrixXd::Zero(6,m_wayPoint.cols()+7);
    s.block(0,0,3,3) = TB;
    s1.block(0,0,3,3) = TB;
    s2.block(0,0,3,3) = TB;
    s3.block(0,0,3,3) = TB;
    s4.block(0,0,3,3) = TB;
    s5.block(0,0,3,3) = TB;

    endP1.block(0,0,3,3) = TB;
    endP2.block(0,0,3,3) = TB;
    Eigen::MatrixXd cons;//
    cons = Eigen::MatrixXd::Ones(3,m_wayPoint.cols());
    cons.row(0)=-cons.row(0)*0.008612; //cons_x
    cons.row(1)=-cons.row(1)*0.296422; //cons_y
    cons.row(2)=-cons.row(2)*0.091263; //cons_z
    m_wayPoint = m_wayPoint+TB*cons;
    s1.block(0,3,3,1) = m_wayPoint.block(0,0,3,1);

    endP1.block(0,3,3,1) = m_wayPoint.block(0,0,3,1);
    endP2.block(0,3,3,1) = m_wayPoint.block(0,0,3,1);
    Eigen::MatrixXd T;
    Eigen::MatrixXd T1;
    Eigen::MatrixXd T2;
    Eigen::MatrixXd T3;
    Eigen::MatrixXd T4;
    Eigen::MatrixXd T5;
    T = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T(2,0)=-0.12; //approach z1
    T1 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T1(1,0)=-0.10; //approach y1
    T2 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T2(2,0)=-0.1; //re_treat z1
    T3 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T3(2,0)=0.24; //approach z2
    T4 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T4(1,0)=0.12; // approach y2
    s1.block(0,3,3,1) = s1.block(0,3,3,1)+TB*T;
    s2.block(0,3,3,1) = s1.block(0,3,3,1)+TB*T1;
    s3.block(0,3,3,1) = s2.block(0,3,3,1)+TB*T3;
    s4.block(0,3,3,1) = s3.block(0,3,3,1)+TB*T4;

    Eigen::VectorXd ref_angel(6,1),angel(6,1);
    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s1,Sangle1))
    {
        qDebug()<<"the start inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,0,6,1)=Sangle1;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s2,Sangle2))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,1,6,1) = Sangle2;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s3,Sangle3))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,2,6,1) = Sangle3;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s4,Sangle4))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,3,6,1) = Sangle4;

    for(int i=0;i<m_wayPoint.cols();i++)
    {
        s.block(0,3,3,1) =m_wayPoint.block(0,i,3,1);
        if (i==0)
        {
            getJointAngle(ref_angel);
            if(!inverseKinematics(ref_angel,s,angel))
            {
                qDebug()<<"the inverse kinematics is not exist";
                return false;
            }
        }
        InverseK.block(0,i+4,6,1)=angel;
    }

    T5 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T5(1,0)=-0.04; //y
    s5.block(0,3,3,1) = s.block(0,3,3,1)+TB*T5;
    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s5,Sangle5))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,5,6,1) = Sangle5;

    endP1.block(0,3,3,1) = s5.block(0,3,3,1)+TB*T1;
    endP2.block(0,3,3,1) = endP1.block(0,3,3,1)+TB*T2;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP1,endangel))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-2,6,1) = endangel;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP2,endangel2))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-1,6,1) = endangel2;

    forwardKinematics(angel,s);
        for(int j=0;j<InverseK.cols();j++)
        {
          IKangel1.append(InverseK(0,j));
          IKangel2.append(InverseK(1,j));
          IKangel3.append(InverseK(2,j));
          IKangel4.append(InverseK(3,j));
          IKangel5.append(InverseK(4,j));
          IKangel6.append(InverseK(5,j));
        }
        IKArray={IKangel1,IKangel2,IKangel3,IKangel4,IKangel5,IKangel6};

        m_strokeIK.append(IKArray);
        qDebug()<<"the stroke inverse is exist";
        return true;
}

int SortFruit::Pick_Approach()
{
    Eigen::MatrixXd stroke_joint(6,m_strokeIK[0].toArray()[0].toArray().size());
    for(int j=0;j<m_strokeIK[0].toArray()[0].toArray().size();j++)
    {
      stroke_joint(0,j)=  m_strokeIK[0].toArray()[0].toArray()[j].toDouble();
      stroke_joint(1,j)=  m_strokeIK[0].toArray()[1].toArray()[j].toDouble();
      stroke_joint(2,j)=  m_strokeIK[0].toArray()[2].toArray()[j].toDouble();
      stroke_joint(3,j)=  m_strokeIK[0].toArray()[3].toArray()[j].toDouble();
      stroke_joint(4,j)=  m_strokeIK[0].toArray()[4].toArray()[j].toDouble();
      stroke_joint(5,j)=  m_strokeIK[0].toArray()[5].toArray()[j].toDouble();
    }
    Eigen::VectorXd angle(6,1);
    getJointAngle(angle);
    move(angle,true);

    int num = 0;
    int a;
    moveline(stroke_joint.col(0),true);
    qDebug()<< "approach 1:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(0),true);
        qDebug()<< "approach 1:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(1),true);
    qDebug()<< "approach 2:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(1),true);
        qDebug()<< "approach 2:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(2),true);
    qDebug()<< "approach 3:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(2),true);
        qDebug()<< "approach 3:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(3),true);
    qDebug()<< "approach 4:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(3),true);
        qDebug()<< "approach 4:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(4),true);
    qDebug()<< "approach 5:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(4),true);
        qDebug()<< "approach 5:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;
//    qDebug()<< a << "\n";
//    if(a!=0&&a!=21300)
//    {
//        return -2;
//    }
    return 0;
}

int SortFruit::Pick_Approach_2()
{
    Eigen::MatrixXd stroke_joint(6,m_strokeIK[0].toArray()[0].toArray().size());
    for(int j=0;j<m_strokeIK[0].toArray()[0].toArray().size();j++)
    {
      stroke_joint(0,j)=  m_strokeIK[0].toArray()[0].toArray()[j].toDouble();
      stroke_joint(1,j)=  m_strokeIK[0].toArray()[1].toArray()[j].toDouble();
      stroke_joint(2,j)=  m_strokeIK[0].toArray()[2].toArray()[j].toDouble();
      stroke_joint(3,j)=  m_strokeIK[0].toArray()[3].toArray()[j].toDouble();
      stroke_joint(4,j)=  m_strokeIK[0].toArray()[4].toArray()[j].toDouble();
      stroke_joint(5,j)=  m_strokeIK[0].toArray()[5].toArray()[j].toDouble();
    }
    Eigen::VectorXd angle(6,1);
    getJointAngle(angle);
    move(angle,true);

    int a;
    int num = 0;
    a = moveline(stroke_joint.col(5),true);
    qDebug()<< "approach 6:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(5),true);
        qDebug()<< "approach 6:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    return 0;
}

int SortFruit::Pick_Retreat()
{
    Eigen::MatrixXd stroke_joint(6,m_strokeIK[0].toArray()[0].toArray().size());
    for(int j=0;j<m_strokeIK[0].toArray()[0].toArray().size();j++)
    {
      stroke_joint(0,j)=  m_strokeIK[0].toArray()[0].toArray()[j].toDouble();
      stroke_joint(1,j)=  m_strokeIK[0].toArray()[1].toArray()[j].toDouble();
      stroke_joint(2,j)=  m_strokeIK[0].toArray()[2].toArray()[j].toDouble();
      stroke_joint(3,j)=  m_strokeIK[0].toArray()[3].toArray()[j].toDouble();
      stroke_joint(4,j)=  m_strokeIK[0].toArray()[4].toArray()[j].toDouble();
      stroke_joint(5,j)=  m_strokeIK[0].toArray()[5].toArray()[j].toDouble();
    }
    Eigen::VectorXd angle(6,1);
    getJointAngle(angle);
    move(angle,true);

    int a;
    int num = 0;
    a = moveline(stroke_joint.col(stroke_joint.cols()-2),true);
    qDebug()<< "retreat 1:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-2),true);
        qDebug()<< "retreat 1:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(stroke_joint.cols()-1),true);
    qDebug()<< "retreat 2:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-1),true);
        qDebug()<< "retreat 2:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    return 0;
}

Eigen::MatrixXd SortFruit::Place_ComputeSample(double initPos[3],Eigen::Quaterniond quaternion,float info[9])
{
    Eigen::MatrixXd T;
    //平移矩阵
    T = Eigen::MatrixXd::Ones(3,m_wayPoint.cols());
    T.row(0)=T.row(0)*initPos[0]; //x
    T.row(1)=T.row(1)*initPos[1]; //y
    T.row(2)=T.row(2)*initPos[2]; //z

    Eigen::MatrixXd p1;
    Eigen::MatrixXd p2;
    p1 = Eigen::MatrixXd::Zero(3,1);
    p1(0,0)=info[3];
    p1(1,0)=info[4];
    p1(2,0)=info[5];
    p2 = Eigen::MatrixXd::Zero(3,1);
    p2(0,0)=info[6];
    p2(1,0)=info[7];
    p2(2,0)=info[8];
    qDebug()<< p1(0,0) << p1(1,0) << p1(2,0) << "\n";
    qDebug()<< m_wayPoint(0,0) << m_wayPoint(1,0) << m_wayPoint(2,0) << "\n";
    qDebug()<< p2(0,0) << p2(1,0) << p2(2,0) << "\n";
    Eigen::Matrix3d  rotateMat = quaternion.toRotationMatrix();
    p1 = rotateMat*p1+T;
    p2 = rotateMat*p2+T;
    m_wayPoint = rotateMat*m_wayPoint+T;
    Eigen::Matrix4d T2;
    Eigen::Matrix3d T3;
    Eigen::MatrixXd T4;
    getFlangePose(T2);
    T3 = T2.block(0,0,3,3);
    T4 = T2.block(0,3,3,1);
    p1 = T3*p1+T4;
    p2 = T3*p2+T4;
    m_wayPoint = T3*m_wayPoint+T4;
    qDebug()<< p1(0,0) << p1(1,0) << p1(2,0) << "\n";
    qDebug()<< m_wayPoint(0,0) << m_wayPoint(1,0) << m_wayPoint(2,0) << "\n";
    qDebug()<< p2(0,0) << p2(1,0) << p2(2,0) << "\n";

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Vector3d vo;
//    Eigen::Vector3d vo(0.0,1.0,0.0);

    v2(0) = p1(0,0)-p2(0,0);
    v2(1) = p1(1,0)-p2(1,0);
    v2(2) = p1(2,0)-p2(2,0);
    vo(0) = T2(0,0);
    vo(1) = T2(1,0);
    vo(2) = T2(2,0);
    v3 = vo.cross(v2);
    v1 = v2.cross(v3);
//    v3 = v2.cross(vo);
//    v1 = v2.cross(v3);
    v1.normalize();
    v2.normalize();
    v3.normalize();
    Place_TB.block(0,0,3,1) = v1;
    Place_TB.block(0,1,3,1) = v2;
    Place_TB.block(0,2,3,1) = v3;
    return m_wayPoint;
}

bool SortFruit::Place_ComputeIK()
{
    QJsonArray IKArray,IKangel1,IKangel2,IKangel3,IKangel4,IKangel5,IKangel6;
    Eigen::Matrix4d s=Eigen::Matrix4d::Identity();
    Eigen::VectorXd Sangle1(6,1),Sangle2(6,1),endangel(6,1),endangel2(6,1),endangel3(6,1),endangel4(6,1);
    Eigen::Matrix4d s1=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d s2=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP1=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP2=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP3=Eigen::Matrix4d::Identity();
    Eigen::Matrix4d endP4=Eigen::Matrix4d::Identity();

    Eigen::MatrixXd InverseK = Eigen::MatrixXd::Zero(6,m_wayPoint.cols()+6);
    s.block(0,0,3,3) = Place_TB;
    s1.block(0,0,3,3) = Place_TB;
    s2.block(0,0,3,3) = Place_TB;

    endP1.block(0,0,3,3) = Place_TB;
    endP2.block(0,0,3,3) = Place_TB;
    endP3.block(0,0,3,3) = Place_TB;
    endP4.block(0,0,3,3) = Place_TB;

    Eigen::MatrixXd cons;//
    cons = Eigen::MatrixXd::Ones(3,m_wayPoint.cols());
    cons.row(0)=cons.row(0)*0.001; //cons_x
    cons.row(1)=cons.row(1)*0.001; //cons_y
    cons.row(2)=-cons.row(2)*0.09; //cons_z
    m_wayPoint = m_wayPoint+Place_TB*cons;

    s1.block(0,3,3,1) = m_wayPoint.block(0,0,3,1);
    endP1.block(0,3,3,1) = m_wayPoint.block(0,0,3,1);

    Eigen::MatrixXd T;
    Eigen::MatrixXd T1;
    Eigen::MatrixXd T2;
    Eigen::MatrixXd T3;
    Eigen::MatrixXd T4;
    Eigen::MatrixXd T5;

    T = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T(1,0)=-0.05; //approach y1
    s1.block(0,3,3,1) = s1.block(0,3,3,1)+Place_TB*T;

    T1 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T1(2,0)=-0.1; //approach z1
    s2.block(0,3,3,1) = s1.block(0,3,3,1)+Place_TB*T1;

    T2 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T2(1,0)=0.12; //retreat y2
    endP1.block(0,3,3,1) = endP1.block(0,3,3,1)+Place_TB*T2;

    T3 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T3(2,0)=0.2; // retreat z2
    endP2.block(0,3,3,1) = endP1.block(0,3,3,1)+Place_TB*T3;

    T4 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T4(1,0)=-0.23; //retreat y3
    endP3.block(0,3,3,1) = endP2.block(0,3,3,1)+Place_TB*T4;

    T5 = Eigen::MatrixXd::Zero(3,m_wayPoint.cols());
    T5(2,0)=-0.25; //retreat z3
    endP4.block(0,3,3,1) = endP3.block(0,3,3,1)+Place_TB*T5;

    Eigen::VectorXd ref_angel(6,1),angel(6,1);

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s2,Sangle2))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,0,6,1) = Sangle2;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,s1,Sangle1))
    {
        qDebug()<<"the start inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,1,6,1)=Sangle1;

    for(int i=0;i<m_wayPoint.cols();i++)
    {
        s.block(0,3,3,1) =m_wayPoint.block(0,i,3,1);
        if (i==0)
        {
            getJointAngle(ref_angel);
            if(!inverseKinematics(ref_angel,s,angel))
            {
                qDebug()<<"the inverse kinematics is not exist";
                return false;
            }
        }
        InverseK.block(0,i+2,6,1)=angel;
    }

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP1,endangel))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-4,6,1) = endangel;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP2,endangel2))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-3,6,1) = endangel2;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP3,endangel3))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-2,6,1) = endangel3;

    getJointAngle(ref_angel);
    if(!inverseKinematics(ref_angel,endP4,endangel4))
    {
        qDebug()<<"the end inverse kinematics is not exist";
        return false;
    }
    InverseK.block(0,InverseK.cols()-1,6,1) = endangel4;

    forwardKinematics(angel,s);
        for(int j=0;j<InverseK.cols();j++)
        {
          IKangel1.append(InverseK(0,j));
          IKangel2.append(InverseK(1,j));
          IKangel3.append(InverseK(2,j));
          IKangel4.append(InverseK(3,j));
          IKangel5.append(InverseK(4,j));
          IKangel6.append(InverseK(5,j));
        }
        IKArray={IKangel1,IKangel2,IKangel3,IKangel4,IKangel5,IKangel6};

        m_strokeIK.append(IKArray);
        qDebug()<<"the stroke inverse is exist";
        return true;
}

int SortFruit::Place_Approach()
{
    Eigen::MatrixXd stroke_joint(6,m_strokeIK[0].toArray()[0].toArray().size());
    for(int j=0;j<m_strokeIK[0].toArray()[0].toArray().size();j++)
    {
      stroke_joint(0,j)=  m_strokeIK[0].toArray()[0].toArray()[j].toDouble();
      stroke_joint(1,j)=  m_strokeIK[0].toArray()[1].toArray()[j].toDouble();
      stroke_joint(2,j)=  m_strokeIK[0].toArray()[2].toArray()[j].toDouble();
      stroke_joint(3,j)=  m_strokeIK[0].toArray()[3].toArray()[j].toDouble();
      stroke_joint(4,j)=  m_strokeIK[0].toArray()[4].toArray()[j].toDouble();
      stroke_joint(5,j)=  m_strokeIK[0].toArray()[5].toArray()[j].toDouble();
    }
    Eigen::VectorXd angle(6,1);
    getJointAngle(angle);
    move(angle,true);

    int a;
    int num = 0;
    a = moveline(stroke_joint.col(0),true);
    qDebug()<< "approach 1:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(0),true);
        qDebug()<< "approach 1:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(1),true);
    qDebug()<< "approach 2:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(1),true);
        qDebug()<< "approach 2:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(2),true);
    qDebug()<< "approach 3:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(2),true);
        qDebug()<< "approach 3:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    return 0;
}

int SortFruit::Place_Retreat()
{
    Eigen::MatrixXd stroke_joint(6,m_strokeIK[0].toArray()[0].toArray().size());
    for(int j=0;j<m_strokeIK[0].toArray()[0].toArray().size();j++)
    {
      stroke_joint(0,j)=  m_strokeIK[0].toArray()[0].toArray()[j].toDouble();
      stroke_joint(1,j)=  m_strokeIK[0].toArray()[1].toArray()[j].toDouble();
      stroke_joint(2,j)=  m_strokeIK[0].toArray()[2].toArray()[j].toDouble();
      stroke_joint(3,j)=  m_strokeIK[0].toArray()[3].toArray()[j].toDouble();
      stroke_joint(4,j)=  m_strokeIK[0].toArray()[4].toArray()[j].toDouble();
      stroke_joint(5,j)=  m_strokeIK[0].toArray()[5].toArray()[j].toDouble();
    }
    Eigen::VectorXd angle(6,1);
    getJointAngle(angle);
    move(angle,true);

    int a;
    int num = 0;
    a = moveline(stroke_joint.col(stroke_joint.cols()-4),true);
    qDebug()<< "retreat 1:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-4),true);
        qDebug()<< "retreat 1:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(stroke_joint.cols()-3),true);
    qDebug()<< "retreat 2:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-3),true);
        qDebug()<< "retreat 2:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(stroke_joint.cols()-2),true);
    qDebug()<< "retreat 3:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-2),true);
        qDebug()<< "retreat 3:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    a = moveline(stroke_joint.col(stroke_joint.cols()-1),true);
    qDebug()<< "retreat 4:" <<  a << "\n";
    while(a!=0)
    {
        sleep(0.1);
        a = moveline(stroke_joint.col(stroke_joint.cols()-1),true);
        qDebug()<< "retreat 4:" << a << "\n";
        num++;
        if (num == 4)
        {
            return -2;
        }
    }
    num = 0;

    return 0;
}

SortFruit::~SortFruit()
{

}
