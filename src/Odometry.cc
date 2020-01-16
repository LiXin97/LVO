//
// Created by lixin04 on 20年1月13日.
//

#include "Odometry.hpp"

namespace LVO
{
    long Odometry::feature_id = 0;

    void Odometry::input_frame(std::shared_ptr<LVO::StereoFrame> &stereoframe)
    {
        cur_frame = stereoframe;
        cur_frame->match_stereo_line();

        tracking();
    }

    void Odometry::tracking()
    {
        if(state == OdoState::UNINIT)
        {
            // 1. 添加双目匹配上的线进系统
            {
                int tri_ok = 0;
                std::map<long, Eigen::Matrix4d> frame_map;
                Eigen::Matrix4d Iden = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d Tlr = cur_frame->get_stereo_param()->Tlr;
                frame_map.emplace( cur_frame->get_StereoId()*2, Iden );
                frame_map.emplace( cur_frame->get_StereoId()*2+1, Tlr );

                std::vector<Eigen::Vector4d> left_ml, right_ml;
                cv::Mat des;
                std::tie( left_ml, right_ml, des ) = cur_frame->get_stereo_match_line_obs();
                for(int i=0;i<left_ml.size();++i)
                {
                    LineFeature lineFeature;
                    lineFeature.insert_ob(cur_frame->get_StereoId(), left_ml[i], right_ml[i]);
                    auto tri_state = lineFeature.tri_two_plane(frame_map);
                    if( std::get<0>(tri_state) == true  ) tri_ok++;

                    SW_features.emplace(feature_id++, lineFeature);
                }

                std::cout << "tri_ok = " << tri_ok << std::endl;
            }
        }
        else if(state == OdoState::OK)
        {

        }
    }
}