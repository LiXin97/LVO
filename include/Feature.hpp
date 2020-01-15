//
// Created by lixin04 on 20年1月14日.
//

#ifndef LVO_FEATURE_HPP
#define LVO_FEATURE_HPP


#include "common.hpp"

namespace LVO{
    class LineFeature3D{
    public:
        LineFeature3D(const Eigen::Vector3d& left, const Eigen::Vector3d& right): LeftPoint(left), RightPoint(right){}
        ~LineFeature3D() = default;

        Eigen::Vector3d getLeftPoint(){return LeftPoint;}
        Eigen::Vector3d getRightPoint(){return RightPoint;}

    private:
        Eigen::Vector3d LeftPoint;
        Eigen::Vector3d RightPoint;
    };

    // TODO 打算左右目相机使用不同的id
    class LineFeatureOb{
    public:
        LineFeatureOb(const Eigen::Vector4d& left, const Eigen::Vector4d& right): LeftOb(left), RightOb(right){ status = Stereo; }
        explicit LineFeatureOb(const Eigen::Vector4d& ob, bool isleft = true)
        {
            if(isleft)
            {
                LeftOb = ob;
                status = Left;
            } else
            {
                RightOb = ob;
                status = Right;
            }
        }
        ~LineFeatureOb() = default;

        enum ObserveStatus {
            Left = 0,    // only left see
            Stereo,           // stereo see
            Right          // only right see
        };  // the status of observe

        Eigen::Vector4d getLeftob(){return LeftOb;}
        Eigen::Vector4d getRightob(){return RightOb;}

    private:
        Eigen::Vector4d LeftOb;
        Eigen::Vector4d RightOb;

        ObserveStatus status = Left;
    };

    class LineFeature{
    public:
        LineFeature() = default;
        ~LineFeature() = default;

        LineFeature3D get3D();

        void insert_ob(long id, LineFeatureOb& ob)
        {
            obs.emplace(id, ob);
        }

        void insert_ob(long id, Eigen::Vector4d& left_ob, Eigen::Vector4d& right_ob)
        {
            LineFeatureOb ob(left_ob, right_ob);
            obs.emplace(id, ob);
        }

        void remove_frame(long id)
        {
            // test
            // TODO update the parameter
            if(id == obs.begin()->first)
            {

            }
            obs.erase(id);
        }

        void keep_frame(const std::set<long>& ids)
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

        std::map<long, LineFeatureOb> get_obs(){return obs;}


    private:
        std::map<long, LineFeatureOb> obs;  // < frame_id, observe >

        Eigen::Matrix<double, 6, 1> plucker;  // plucker in 3D
        // descriptor
    };
}

#endif //LVO_FEATURE_HPP
