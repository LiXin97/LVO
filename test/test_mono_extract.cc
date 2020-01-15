//
// Created by lixin04 on 20年1月14日.
//

#include "StereoFrame.hpp"

int main()
{
    LVO::MonoParam param;
    cv::Mat img = cv::imread( "/home/lixin04/data/KITTI/data_odometry/dataset/sequences/04/image_0/000181.png", 0 );
    LVO::Frame frame( 0, 0, img, param );
    cv::Mat show = frame.get_extract_img();
    cv::imshow(" show ", show);
    cv::waitKey();
    return 0;
}