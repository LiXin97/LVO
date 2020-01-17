//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_ODOMETRY_HPP
#define LVO_ODOMETRY_HPP

#include "common.hpp"
#include "Feature.hpp"
#include "StereoFrame.hpp"

#include "optimi/line_projection_factor.hpp"
#include "optimi/line_parameterization.hpp"
#include "optimi/pose_local_parameterization.hpp"

namespace LVO
{
    class Odometry{
    public:
        Odometry()= default;
        ~Odometry() = default;

        enum OdoState
        {
            UNINIT = -1,
            OK,
            LOST
        };

        void tracking();

        void track_LastKeyframe();
        bool track_motion_mode();
        bool track_sw();

        void optimization_SW();
        void optimization_curpose( Eigen::Matrix4d& Twc, const Eigen::Matrix4d& Tlr, const std::map<int, int>& match );

        void update_velocity();

        bool need_keyframe();

        void add_keyframe();

        std::map< int, int > matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr = 0.75);


        // DeBUG
        void show_match(
                std::map<int, int> &match_result,
                std::vector<Eigen::Vector4d> &cur_lines_pixel,
                std::vector<Eigen::Vector4d> &last_lines_pixel,
                std::vector< long >& cur_frame_id,
                std::vector< long >& last_frame_id );


        void remove_frame(long id)
        {
            auto iter = SW_features.begin();
            for(auto ite = iter; iter != SW_features.end(); ite = iter)
            {
                iter++;
                ite->second.remove_frame(id);
                if(ite->second.num_obs() < 1)
                {
                    SW_features.erase(ite);
                }
//                std::cout << " SW_features.size() = " << SW_features.size() << std::endl;
            }

            //TODO remove featrue add to Old_3D
        }

        void keep_SW_frame( const std::set<long>& ids )
        {
            auto iter = SW_features.begin();
            for(auto ite = iter; iter != SW_features.end(); ite = iter)
            {
                iter++;
                ite->second.keep_frame(ids);
                if(ite->second.num_obs() < 1)
                {
                    SW_features.erase(ite);
                }
//                std::cout << " SW_features.size() = " << SW_features.size() << std::endl;
            }

            //TODO remove featrue add to Old_3D
        }

        void insert_ob(long feature_id, long frame_id, Eigen::Vector4d& observe)
        {
            if(SW_features.count(feature_id) == 0)
            {
                LineFeature feature;
                feature.insert_ob(frame_id, observe);
                SW_features.emplace(feature_id, feature);
            } else
            {
                SW_features[feature_id].insert_ob(frame_id, observe);
            }
        }

        void print_SWobs()
        {
            std::cout <<  "---------------------" << std::endl;
            for(auto &feature:SW_features)
            {
                auto obs = feature.second.get_obs();
                std::cout << feature.first << " < ---";
                for(auto &ob:obs)
                {
                    std::cout << " " << ob.first;
                }
                std::cout << std::endl;
            }
            std::cout <<  "---------------------" << std::endl;
        }

        void input_frame(std::shared_ptr<StereoFrame>& stereoframe);

        OdoState get_state(){return state;}

        void update_cur_frame_feature_id( const std::map<int, int>& match_opti );

    private:
        // 滑窗存的是单目的frame
//        std::map< long, std::shared_ptr<Frame> > SW_frames;   // < frame_id, Frame >
//        std::map< long, std::shared_ptr<Frame> > Old_frames;  // < frame_id, Frame >

        std::map< long, Eigen::Matrix4d > SW_frames;   // < frame_id, Frame >
        std::map< long, Eigen::Matrix4d > Old_frames;  // < frame_id, Frame >
        // shared_ptr
        std::shared_ptr<StereoFrame> last_key_frame;
        std::shared_ptr<StereoFrame> last_frame;
        std::shared_ptr<StereoFrame> cur_frame;

        OdoState state = OdoState::UNINIT;

        Eigen::Matrix4d motion_velocity;

        static long feature_id;

        OdoParam odoParam;

        std::map< long, LineFeature > SW_features;  // < feature_id, observes >
        std::map< long, LineFeature3D > Old_3Dfeatures; // < feature_id, 3D points left right >
    };

}

#endif //LVO_ODOMETRY_HPP
