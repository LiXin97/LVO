//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_ODOMETRY_HPP
#define LVO_ODOMETRY_HPP

#include "common.hpp"
#include "StereoFrame.hpp"

namespace LVO{

    class FeaturesOb{
    public:
        FeaturesOb(const Eigen::Vector4d& left, const Eigen::Vector4d& right): LeftOb(left), RightOb(right){}
        ~FeaturesOb() = default;

        Eigen::Vector4d getLeftob(){return LeftOb;}
        Eigen::Vector4d getRightob(){return RightOb;}

    private:
        Eigen::Vector4d LeftOb;
        Eigen::Vector4d RightOb;
    };

    class FeaturesObs{
    public:
        FeaturesObs() = default;
        ~FeaturesObs() = default;

        void insert_ob(long id, FeaturesOb& ob)
        {
            obs.emplace(id, ob);
        }

        void insert_ob(long id, Eigen::Vector4d& left_ob, Eigen::Vector4d& right_ob)
        {
            FeaturesOb ob(left_ob, right_ob);
            obs.emplace(id, ob);
        }

        void remove_frame(long id)
        {
            // TODO update the parameter
            obs.erase(id);
        }

        void keep_frame(std::set<long> ids)
        {
            for(auto &ob:obs)
            {
                if(ids.count(ob.first) < 1)
                {
                    obs.erase(ob.first);
                }
            }
        }

        int num_obs()
        {
            return obs.size();
        }



//    private:
        std::map<long, FeaturesOb> obs;  // < frame_id, observe >

        // TODO Discrap
    };

    class Features3D{
    public:
        Features3D(const Eigen::Vector3d& left, const Eigen::Vector3d& right): LeftPoint(left), RightPoint(right){}
        ~Features3D() = default;

        Eigen::Vector3d getLeftPoint(){return LeftPoint;}
        Eigen::Vector3d getRightPoint(){return RightPoint;}

    private:
        Eigen::Vector3d LeftPoint;
        Eigen::Vector3d RightPoint;
    };

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

        void keep_SW_frame( std::set<long> ids )
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
                FeaturesObs obs;
                obs.insert_ob(frame_id, left_ob, right_ob);
                SW_features.emplace(feature_id, obs);
            } else
            {
                it->second.insert_ob(frame_id, left_ob, right_ob);
            }
        }

        void print_SWobs()
        {
            std::cout <<  "---------------------" << std::endl;
            for(auto &obs:SW_features)
            {
                std::cout << obs.first << " < ---";
                for(auto &ob:obs.second.obs)
                {
                    std::cout << " " << ob.first;
                }
                std::cout << std::endl;
            }
            std::cout <<  "---------------------" << std::endl;
        }

        void insert_frame(StereoFrame& frame);

    private:
        std::map< long, StereoFrame > SW_frames;

        std::map< long, FeaturesObs > SW_features;  // < feature_id, observes >
        std::map< long, Features3D > Old_3Dfeatures;
    };

}

#endif //LVO_ODOMETRY_HPP
