#include <iostream>
#include <opencv2/imgcodecs.hpp>

class sample
{
public:
	int parameter();
    bool ImgAndCloud();
    std::vector<Eigen::Vector4f> seg_cloud(float r1,float c1,float r2,float c2);

private:
	cv::Mat to8U(const cv::Mat& src);
};
