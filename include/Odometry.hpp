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

        enum OdoState
        {
            UNINIT = -1,
            OK,
            LOST
        };

        void tracking();

        void track_LastKeyframe();
        void track_SW();

        void optimization_SW();
        void optimization_CurPose();


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

    private:
        // 滑窗存的是单目的frame
        std::map< long, std::shared_ptr<Frame> > SW_frames;   // < frame_id, Frame >
//        std::map< long, std::shared_ptr<Frame> > Old_frames;  // < frame_id, Frame >

        // shared_ptr
        std::shared_ptr<StereoFrame> last_key_frame;
        std::shared_ptr<StereoFrame> cur_frame;

        OdoState state = OdoState::UNINIT;

        static long feature_id;

        std::map< long, LineFeature > SW_features;  // < feature_id, observes >
        std::map< long, LineFeature3D > Old_3Dfeatures; // < feature_id, 3D points left right >
    };

}

#endif //LVO_ODOMETRY_HPP
