//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_STEREOFRAME_HPP
#define LVO_STEREOFRAME_HPP

#include "common.hpp"
#include "Param.hpp"
#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>

namespace LVO
{
    class Frame{
    public:
        Frame(
                long _FrameId,
                long _TimeStamp,
                const cv::Mat& Img,
                const MonoParam& param
                );

        Frame();
        ~Frame() = default;

        void extract_line_lsd();

        cv::Mat get_extract_img();

        cv::Mat get_img(){return img.clone();}

        void setTwc(const Eigen::Matrix4d& Twc_input){ Twc = Twc_input; }

        Eigen::Matrix4d getTwc(){return Twc;}

        std::vector<cv::line_descriptor::KeyLine> get_line_extract(){return keyline;}
        cv::Mat get_lbd(){return keylbd_descr.clone();};

    private:
        long frame_id;
        long timestamp;

        cv::Mat img;
        cv::Mat img_src;

        cv::Mat lbd_descr, keylbd_descr;
        std::vector<cv::line_descriptor::KeyLine> line, keyline;

        Eigen::Matrix4d Twc;

        MonoParam mono_param;
    };

    class StereoFrame{
    public:
        StereoFrame(
                long _StereoframeId,
                long _TimeStamp,
                const cv::Mat& _LeftImg,
                const MonoParam& leftparam,
                const cv::Mat& _RightImg,
                const MonoParam& rightparam,
                const Eigen::Matrix4d& Trl,
                const StereoMatchParam& stereoparam
                );
        ~StereoFrame() = default;

        void match_stereo_line();

        std::tuple<cv::Mat ,cv::Mat> get_stereo_match();

    private:
        long TimeStamp;

        Frame left, right;

        StereoMatchParam stereo_param;

        std::vector<cv::line_descriptor::KeyLine> left_match_lines, right_match_lines;

        std::vector<cv::line_descriptor::KeyLine> left_un_match_lines, right_un_match_lines;

        cv::Mat lbd_descr, lbd_left_descr_remiand, lbd_right_descr_remiand;

        Eigen::Matrix4d Trl;
    };
}

#endif //LVO_STEREOFRAME_HPP
