#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/persistence.hpp>
#include "json/json.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>

#include "sample.h"

inline bool isApprox0(double d, double epsilon = DBL_EPSILON) {return std::fabs(d) <= epsilon;}

cv::Mat sample::to8U(const cv::Mat& src) 
{
	if (src.empty()) return {};
	double minV, maxV;
	cv::minMaxLoc(src, &minV, &maxV);
	cv::Mat dst;
	src.convertTo(dst, CV_8U, isApprox0(maxV) ? 1 : 255.0 / maxV);
	return dst;
}

//In this sample, we will show how to use a camera to take a 2d color image and save it on the disk
bool sample::ImgAndCloud()
{
	//Before we do anything, we always need to connect to camera by ip.
	CameraClient camera;
	std::string error;
	// Camera ip should be modified to actual ip address.
        const std::string cameraIp = "192.168.0.6";
    if (!camera.connect(cameraIp)) return false; //return -1 if connection to camera fails
	std::cout 
		<< "Camera ID: " << camera.getCameraId() << std::endl
		<< "Version: " << camera.getCameraVersion() << std::endl 
		<< "Color Image Size: " << camera.getColorImgSize() << std::endl
		<< "Depth Image Size: " << camera.getDepthImgSize() << std::endl; //get and print some information about camera device

	cv::Mat color = camera.captureColorImg(); //capture a 2d image and it will be stored as cv matrix
	if (color.empty()) std::cout << "The color image is empty!" << std::endl;
	else
	{
                cv::imwrite("/home/esir/Sort2/pic/gun_sun_2.jpg", color); //save the color image to the disk
	}

	cv::Mat depth = camera.captureDepthImg(); //capture a depth image and it will be stored as cv matrix
	if (depth.empty()) std::cout << "The depth image is empty!" << std::endl;
	else
	{
        cv::imwrite("/home/esir/Sort2/pic/gun_sun_2.png", depth);
	}

	std::cout << "Generating point cloud image" << std::endl;
	const pcl::PointCloud<pcl::PointXYZRGB> rgbCloud = camera.captureRgbPointCloud();
        if(rgbCloud.points.size() > 0){
            pcl::io::savePCDFileASCII("/home/esir/Sort2/pic/gun_sun_2.pcd",rgbCloud);
            return true;
        }
        else {
            return true;
        }


}
std::vector<Eigen::Vector4f> sample::seg_cloud(float r1,float c1,float r2,float c2){
    //创建一个容器存放3个点
    std::vector<Eigen::Vector4f> point_three;

    Eigen::Vector4f v1 = Eigen::Vector4f::Zero();
    Eigen::Vector4f v2 = Eigen::Vector4f::Zero();
    Eigen::Vector4f v3 = Eigen::Vector4f::Zero();
    std::vector<Eigen::Vector4f> point_fause ; //异常情况返回此值
    point_fause.push_back(v1);
    point_fause.push_back(v2);
    point_fause.push_back(v3);

    pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
    if (pcl::io::loadPCDFile("/home/esir/Sort2/pic/gun_sun_2.pcd",rgbCloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
      {
        PCL_ERROR ("Couldn't read file\n");
        //return (-1);
      }

    //float r1 =661, c1 = 651,r2 =1024, c2 =779;
	//创建一个rgbCloud对应点云指针 用于将直线显示在原点云中
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud_Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	rgbCloud_Ptr = rgbCloud.makeShared();   
	
	//过滤框外点云，将框外点云值置为0 (传过来(r1,c1),(r2,c2)),rgbCloud.(c,r)对应于rgb.(r,c)
	for(float r=0; r < 1024 ; r++) {
		for ( float c = 0 ; c < 1280 ; c++){
			if( r < c1 || r > c2 || c < r1 || c > r2){
				pcl::PointXYZRGB p;
				p.x = 0;
				p.y = 0;
				p.z = 0;
                                p.r = 0;
				p.g = 0;
				p.b = 0;
				rgbCloud.at(c,r) = p;
			}
		}
	}
	/*条件滤波：保留Z>0的点*/
	//创建条件对象
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
	rgbCloudPtr = rgbCloud.makeShared();   //点云对应的指针对象
	/*
	//比较条件符号说明:
	//GT greater than
	//EQ equal
	//LT less than
	//GE greater than or equal
	//LE less than or equal
	*/
	//添加比较条件：z>0.0
	range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 					pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));



	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;	//创建条件滤波器对象
	condrem.setCondition (range_cond);				//设置条件
	condrem.setInputCloud (rgbCloudPtr);					//设置输入点云
	condrem.setKeepOrganized(true);					//设置保持点云的结构
	condrem.filter (*cloud_filtered);				//执行滤波

	//std::cout << cloud_filtered->size() << std::endl;
	/*0点变为NAN点，清楚NaN点*/
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);

    //如果点云数量小于10,则按异常处理
    if(cloud_filtered->points.size() < 10){
        return point_fause;
    }

	//std::cout << cloud_filtered->size() << std::endl;
    pcl::io::savePCDFileASCII("/home/esir/Sort2/pic/target.pcd",*cloud_filtered);
    
	/*计算中心点xyz*/
	
	  // 创建存储点云重心的对象
   	 Eigen::Vector4f centroid;
    
   	 pcl::compute3DCentroid(*cloud_filtered, centroid);

   	 std::cout << "点云重心坐标: ("
             	 << centroid[0] << ", "
             	 << centroid[1] << ", "
              	 << centroid[2] << ")." << std::endl;	
     point_three.push_back(centroid);
	
	//保存为pcd
	/*×××××××××××××将点云切分为两部分，分别计算重心并求方向向量××××××××××××××××××*/
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond_1 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond_2 (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//添加比较条件1：x<centroid[0]
	range_cond_1->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 			  	        		pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, centroid[0])));
	
	//添加比较条件1：x>centroid[0]
	range_cond_2->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new 				   pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, centroid[0])));
	


	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_1;	
	condrem_1.setCondition (range_cond_1);				
	condrem_1.setInputCloud (cloud_filtered);					
	condrem_1.setKeepOrganized(true);					
	condrem_1.filter (*cloud_filtered_1);				

	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem_2;	
	condrem_2.setCondition (range_cond_2);				
	condrem_2.setInputCloud (cloud_filtered);					
	condrem_2.setKeepOrganized(true);					
	condrem_2.filter (*cloud_filtered_2);				

	//去除NaN点
	std::vector<int> mapping_1;
	pcl::removeNaNFromPointCloud(*cloud_filtered_1, *cloud_filtered_1, mapping_1);
	std::vector<int> mapping_2;
	pcl::removeNaNFromPointCloud(*cloud_filtered_2, *cloud_filtered_2, mapping_2);

    //如果点云数量小于10,则按异常处理
    if(cloud_filtered_1->points.size() < 10 || cloud_filtered_1->points.size() < 10){
        return point_fause;
    }

    pcl::io::savePCDFileASCII ("/home/esir/Sort2/pic/target_left.pcd", *cloud_filtered_1);
    pcl::io::savePCDFileASCII ("/home/esir/Sort2/pic/target_right.pcd", *cloud_filtered_2);
	//计算cloud_filtered_1和cloud_filtered_2的重心坐标
	Eigen::Vector4f centroid_1;

	pcl::compute3DCentroid(*cloud_filtered_1, centroid_1);

	std::cout << "第一块点云的重心坐标: ("
	      << centroid_1[0] << ", "
	      << centroid_1[1] << ", "
	      << centroid_1[2] << ")." << std::endl;

	Eigen::Vector4f centroid_2;

	pcl::compute3DCentroid(*cloud_filtered_2, centroid_2);

	std::cout << "第二块点云的重心坐标: ("
	      << centroid_2[0] << ", "
	      << centroid_2[1] << ", "
	      << centroid_2[2] << ")." << std::endl;
    point_three.push_back(centroid_1);
    point_three.push_back(centroid_2);
    /*方向向量转旋转矩阵
    Eigen::Vector3f v;
    v[0] = centroid_2[0] - centroid_1[0];
    v[1] = centroid_2[1] - centroid_1[1];
    v[2] = centroid_2[2] - centroid_1[2];
    std::cout << "方向向量为:" << v << std::endl;


        //以y轴为参考向量
        float alpha = atan2(v(2) , v(0)) ;
        float beta = atan2(v(1),sqrt(v(0)*v(0) + v(2)*v(2) ));
        Eigen::Matrix3f m;
        Eigen::Vector3f j;
        j(0) = cos(alpha)*cos(beta) ;
        j(1) = sin(beta) ;
        j(2) = sin(alpha)*cos(beta) ;
        Eigen::Vector3f i;
        i(0) = sin(alpha) ;
        i(1) =  0;
        i(2) = -cos(alpha) ;
        Eigen::Vector3f k ;
        k(0) = cos(alpha)*sin(beta)  ;
        k(1) = -cos(beta) ;
        k(2) = sin(alpha)*sin(beta) ;

        m(0, 0) = i(0);
        m(0, 1) = j(0);
        m(0, 2) = k(0);

        m(1, 0) = i(1);
        m(1, 1) = j(1);
        m(1, 2) = k(1);

        m(2, 0) = i(2);
        m(2, 1) = j(2);
        m(2, 2) = k(2);

    std::cout << "旋转矩阵为:" << m << std::endl;
    //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Vector3f euler = m.eulerAngles(2,1,0);
    std::cout << "欧拉角ZYX为：" << euler<<std::endl;

        Eigen::Matrix4f T_pose;
        T_pose << m(0, 0), m(0, 1) ,m(0, 2), centroid[0],
                m(1, 0), m(1, 1),m(1, 2), centroid[1],
                m(2, 0) ,m(2, 1),m(2, 2)  ,centroid[2],
                0.00000 ,0.0000,0.00000,1.0000;
        std::cout << "6d位姿为:" << T_pose << std::endl;
        */

        pcl::PointXYZRGB point;


        //cloud->points.resize (100000 * 1);



        for(float i = -5 ; i < 5 ; i += 10/10000.0){
            point.x = i * (centroid_1[0]-centroid_2[0]) + centroid_1[0];
            point.y = i * (centroid_1[1]-centroid_2[1]) + centroid_1[1];
            point.z = i * (centroid_1[2]-centroid_2[2]) + centroid_1[2];
            point.r = 0;
            point.g = 0;
            point.b = 0;

            rgbCloud_Ptr->push_back(point);
        }
        //两点的方向向量显示在原点云中
        pcl::io::savePCDFileASCII ("look.pcd", *rgbCloud_Ptr);
    /*
        pcl::visualization::PCLVisualizer viewer("11");

        // 1.We add the point cloud to the viewer and pass the color handler
        //viewer.addPointCloud(source_cloud,  "original_cloud");

        // 2.Define R,G,B colors for the point cloud
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color(rgbCloud_Ptr, 255, 255, 255);
        viewer.addPointCloud(rgbCloud_Ptr, source_cloud_color, "original_cloud");

        //坐标系的三维轴添加到屏幕的(0, 0, 0)处
        //viewer.addCoordinateSystem(1.0, "original_cloud", 0);
        viewer.setBackgroundColor(0.1, 0.1, 0, 0); // Setting background to a dark grey设置背景颜色，默认黑色
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");//渲染属性

        //viewer.setPosition(400, 200); // Setting visualiser window position窗口弹出位置

        // 将两个点连线

        pcl::PointXYZ temp1;
        temp1.x = centroid_1[0];
        temp1.y = centroid_1[1];
        temp1.z = centroid_1[2];
        pcl::PointXYZ temp2 ;
        temp2.x = centroid_2[0];
        temp2.y = centroid_2[1];
        temp2.z = centroid_2[2];
        //viewer.addLine(temp1, temp2, "line0");
        // 同样可以设置线的颜色，
        viewer.addLine(temp1, temp2, 255 , 0 , 0 , "line0");

        while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce();
        }
    */
    /*
        //四元数转旋转矩阵
    Eigen::Quaterniond quaternion(0.700130,-0.055469,-0.042420,-0.710593);
    Eigen::Matrix3d  R_cal = quaternion.toRotationMatrix();
    std::cout<<"标定旋转矩阵："<<R_cal<<std::endl;
    Eigen::Vector3d euler_cal = R_cal.eulerAngles(2,1,0);
    std::cout << "标定欧拉角ZYX为：" << euler_cal<<std::endl;
    //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Matrix3f T_test;
    T_test << 0.0285828, 0.0122726 ,0,
                0.0122726, -0.0285828,0,
                -0.00216188 ,0,-0.0009 ;

    Eigen::Vector3f euler_test = T_test.eulerAngles(2,1,0);
    Eigen::Vector3f euler_test_2 ;
    euler_test_2[0] = euler_test[0]*180/3.14;
    euler_test_2[1] = euler_test[1]*180/3.14;
    euler_test_2[2] = euler_test[2]*180/3.14;
    std::cout << "测试欧拉角ZYX：" << euler_test_2<<std::endl;
    */
        
        
        return point_three;
}


