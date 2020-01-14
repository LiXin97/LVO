//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_ODOMETRY_HPP
#define LVO_ODOMETRY_HPP

#include "common.hpp"
#include "Feature.hpp"
#include "StereoFrame.hpp"

namespace LVO{

    class Odometry{
    public:
        Odometry()= default;
        ~Odometry() = default;

        void track_LastKeyframe();
        void track_SW();

        void optimization_SW();
        void optimization_CurPose();


        void remove_frame(long id)
        {
            for(auto &feature:SW_features)
            {
                feature.second.remove_frame(id);
                if(feature.second.num_obs() < 1)
                    SW_features.erase(feature.first);
            }

            //TODO remove featrue add to Old_3D
        }

        void keep_SW_frame( const std::set<long>& ids )
        {
            for(auto &feature:SW_features)
            {
                feature.second.keep_frame(ids);
                if(feature.second.num_obs() < 1)
                    SW_features.erase(feature.first);
            }

            //TODO remove featrue add to Old_3D
        }

        void insert_ob(long feature_id, long frame_id, Eigen::Vector4d& left_ob, Eigen::Vector4d& right_ob)
        {
            auto it = SW_features.find(feature_id);
            if(it == SW_features.end())
            {
                LineFeature feature;
                feature.insert_ob(frame_id, left_ob, right_ob);
                SW_features.emplace(feature_id, feature);
            } else
            {
                it->second.insert_ob(frame_id, left_ob, right_ob);
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

        void input_frame(StereoFrame& frame);

    private:
        std::map< long, StereoFrame > SW_frames;   // < frame_id, Frame >
        std::map< long, StereoFrame > Old_frames;  // < frame_id, Frame >

        std::map< long, LineFeature > SW_features;  // < feature_id, observes >
        std::map< long, LineFeature3D > Old_3Dfeatures; // < feature_id, 3D points left right >
    };

}

#endif //LVO_ODOMETRY_HPP
