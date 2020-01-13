//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_STEREOFRAME_HPP
#define LVO_STEREOFRAME_HPP

#include "common.hpp"

namespace LVO
{
    class StereoFrame{
    public:
        StereoFrame(
                const long _Id,
                const long _TimeStamp,
                const cv::Mat& _LeftImg,
                const cv::Mat& _RightImg,
                const Eigen::Matrix4d& T);
        ~StereoFrame() = default;

    private:
        long FrameId;
        long TimeStamp;

        cv::Mat LeftImg;
        cv::Mat RightImg;

        Eigen::Matrix4d Twc;
    };
}

#endif //LVO_STEREOFRAME_HPP
