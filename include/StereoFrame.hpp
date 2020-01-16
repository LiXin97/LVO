//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_STEREOFRAME_HPP
#define LVO_STEREOFRAME_HPP

#include "common.hpp"
#include "Param.hpp"
#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>
#include "Feature.hpp"

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

        void set_Twc(const Eigen::Matrix4d& Twc_input){ Twc = Twc_input; }

        Eigen::Matrix4d get_Twc(){return Twc;}

        std::vector<cv::line_descriptor::KeyLine> get_line_extract(){return keyline;}
        cv::Mat get_lbd(){return keylbd_descr.clone();};

        long get_id(){return frame_id;}

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

        void set_Twc(const Eigen::Matrix4d& Twc_input)
        {
            left->set_Twc(Twc_input);
            Eigen::Matrix4d Trl = stereo_param->Tlr;
            right->set_Twc( Twc_input*Trl );
        }

        std::tuple< Eigen::Matrix4d, Eigen::Matrix4d > get_Twc(){return std::make_tuple( left->get_Twc(), right->get_Twc() );}

        std::vector< LineFeature > get_obs(){return all_obersves;}

        void set_lineid( std::vector< long >& line_id ){ all_line_id = line_id; }
        
        std::tuple< std::vector< long >, std::vector< LineFeature > > get_id_obs()
        {
            return std::make_tuple( all_line_id, all_obersves );
        }

    private:
        long TimeStamp;

        long StereoFrameId;

        std::shared_ptr< Frame > left, right;

        std::shared_ptr< StereoMatchParam > stereo_param;

        std::vector< long > all_line_id;
        std::vector< LineFeature > all_obersves;

        std::vector<cv::line_descriptor::KeyLine> left_match_lines, right_match_lines;

        Eigen::Matrix4d Trl;
    };
}

#endif //LVO_STEREOFRAME_HPP
