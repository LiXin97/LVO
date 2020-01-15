#ifndef STEREO_H
#define STEREO_H

#include "opencv2/opencv.hpp"

namespace svo {
class Stereo
{
  cv::Mat R1_, P1_, R2_, P2_;
  cv::Rect roi1_, roi2_;
  cv::Mat Q_;

  cv::Mat map11_, map12_, map21_, map22_;

  int rows_,cols_;
  double fx_,fy_,cx_,cy_;    // cam params after stereo rectify
  double bf_;                          // stereo_baseline * f

public:

  Stereo(std::string stereoParam);

  void Rectifyimg(cv::Mat imLeft, cv::Mat imRight, cv::Mat &imLeftRect, cv::Mat &imRightRect);

  inline double fx(){ return fx_;}
  inline double fy(){ return fy_;}
  inline double cx(){ return cx_;}
  inline double cy(){ return cy_;}
  inline double bf(){ return bf_;}
  inline int rows(){return rows_;}
  inline int cols(){return cols_;}

};

}
#endif // STEREO_H
