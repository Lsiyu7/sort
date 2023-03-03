#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include "json/json.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include "sample.h"

int main(int argc, char *argv[])
{
     //类实现
 	/*
	sample sp;
	
    //拍摄图片
	sp.ImgAndCloud();

    //进行分割,返回一个Eigen::Vector4f类型容器，第一个值代表点云重心，后两个分别为第一第二块点云重心
    std::vector<Eigen::Vector4f> seg_cloud(r1,c1,r2,c2);
	*/
    sample sp;
    sp.seg_cloud(661,651,1024,779);

    return 0;
}
