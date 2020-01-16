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
                const std::shared_ptr< MonoParam >& param
                );

        Frame();
        ~Frame() = default;

        void extract_line_lsd();

        cv::Mat get_extract_img();

        cv::Mat get_img(){return img.clone();}

        std::shared_ptr< MonoParam > get_monoparam(){return mono_param;}

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

        std::shared_ptr< MonoParam > mono_param;
    };

    class StereoFrame{
    public:
        StereoFrame(
                long _StereoframeId,
                long _TimeStamp,
                const cv::Mat& _LeftImg,
                const std::shared_ptr< MonoParam >& leftparam,
                const cv::Mat& _RightImg,
                const std::shared_ptr< MonoParam >& rightparam,
                const std::shared_ptr< StereoMatchParam >& stereoparam
                );
        ~StereoFrame() = default;

        void match_stereo_line();

        std::tuple<cv::Mat ,cv::Mat> get_stereo_match();

        std::shared_ptr< Frame > get_left_frame(){return left;}
        std::shared_ptr< Frame > get_right_frame(){return right;}

        std::shared_ptr< StereoMatchParam > get_stereo_param(){return stereo_param;}

        long get_StereoId(){return StereoFrameId;}

        std::tuple< std::vector<cv::line_descriptor::KeyLine>, std::vector<cv::line_descriptor::KeyLine>, cv::Mat > get_stereo_match_line()
        {
            return std::make_tuple( left_match_lines, right_match_lines, lbd_descr );
        }

        std::tuple< std::vector<cv::line_descriptor::KeyLine>, std::vector<cv::line_descriptor::KeyLine>, cv::Mat, cv::Mat > get_remains_lines()
        {
            return std::make_tuple( left_un_match_lines, right_un_match_lines, lbd_left_descr_remains, lbd_right_descr_remains );
        }

        std::tuple< std::vector<Eigen::Vector4d>, std::vector<Eigen::Vector4d>, cv::Mat > get_stereo_match_line_obs()
        {
            return std::make_tuple( left_match_obs, right_match_obs, lbd_descr );
        }

    private:
        long TimeStamp;

        long StereoFrameId;

        std::shared_ptr< Frame > left, right;

        std::shared_ptr< StereoMatchParam > stereo_param;

        std::vector<cv::line_descriptor::KeyLine> left_match_lines, right_match_lines;

        std::vector<cv::line_descriptor::KeyLine> left_un_match_lines, right_un_match_lines;

        std::vector<Eigen::Vector4d> left_match_obs, right_match_obs;

        std::vector<Eigen::Vector4d> left_un_match_obs, right_un_match_obs;

        cv::Mat lbd_descr, lbd_left_descr_remains, lbd_right_descr_remains;

        Eigen::Matrix4d Trl;
    };
}

#endif //LVO_STEREOFRAME_HPP
