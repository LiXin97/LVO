//#include "svo/stereo.h"
#include "stereo.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "iostream"
namespace svo {

Stereo::Stereo(std::string stereoParam)
{

  cv::FileStorage fSettings(stereoParam.c_str(), cv::FileStorage::READ);
  if(!fSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
      return ;
  }

  cols_ = fSettings["Camera.width"];
  rows_ = fSettings["Camera.height"];
  cv::Size img_size(cols_,rows_);

  cv::Mat K_l,K_r,D_l,D_r;
  fSettings["LEFT.K"] >> K_l;
  fSettings["RIGHT.K"] >> K_r;

  fSettings["LEFT.D"] >> D_l;
  fSettings["RIGHT.D"] >> D_r;

  cv::Mat omega,T,R;
  fSettings["RIGHT_TO_LEFT.R"] >> R;
  //exfs["omega"] >> omega;
  fSettings["RIGHT_TO_LEFT.T"] >> T;
  //cv::Rodrigues(omega,R);

  cv::stereoRectify( K_l, D_l, K_r, D_r, img_size, R, T, R1_, R2_, P1_, P2_, Q_, CV_CALIB_ZERO_DISPARITY, 0, img_size, &roi1_, &roi2_ );

  fx_ = P2_.ptr<double>(0)[0];
  fy_ = P2_.ptr<double>(1)[1];
  cx_ = P2_.ptr<double>(0)[2];
  cy_ = P2_.ptr<double>(1)[2];
  bf_ = - P2_.ptr<double>(0)[3];

  /*
  std::cout<<"R1: "<<R1_<<std::endl;
  std::cout<<"P1: "<<P1_<<std::endl;
  std::cout<<"R2: "<<R2_<<std::endl;
  std::cout<<"P2: "<<P2_<<std::endl;
  std::cout<<"Q:"<<Q_<<std::endl;
*/

  cv::initUndistortRectifyMap(K_l, D_l, R1_, P1_, img_size, CV_32F, map11_, map12_);
  cv::initUndistortRectifyMap(K_r, D_r, R2_, P2_, img_size, CV_32F, map21_, map22_);


}

void Stereo::Rectifyimg(cv::Mat imLeft, cv::Mat imRight, cv::Mat &imLeftRect, cv::Mat &imRightRect)
{

  if(imLeft.empty())
  {
      std::cerr <<  "Failed to load imLeft "<< std::endl;
      return ;
  }

  if(imRight.empty())
  {
      std::cerr <<  "Failed to load imRight "<< std::endl;
      return ;
  }

  cv::remap(imLeft, imLeftRect, map11_, map12_, CV_INTER_LINEAR); //CV_INTER_LINEAR
  cv::remap(imRight, imRightRect, map21_, map22_,CV_INTER_LINEAR);

}

}
