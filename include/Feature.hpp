//
// Created by lixin04 on 20年1月14日.
//

#ifndef LVO_FEATURE_HPP
#define LVO_FEATURE_HPP


#include "common.hpp"
#include "line_uti/line_geometry.hpp"

namespace LVO{
    class Frame;

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

    class LineFeatureOb{
    public:
        LineFeatureOb(const Eigen::Vector4d& ob)
        {
            observe4d = ob;
        }
        LineFeatureOb() = default;
        ~LineFeatureOb() = default;

        Eigen::Vector4d get4dob(){return observe4d;}
        Eigen::Vector3d get3dob0(){return Eigen::Vector3d(observe4d(0), observe4d(1), 1.);}
        Eigen::Vector3d get3dob1(){return Eigen::Vector3d(observe4d(2), observe4d(3), 1.);}

    private:
        Eigen::Vector4d observe4d;
    };

    // 这种双目的类，好像不好用
/*    class LineFeatureOb{
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
    };*/

    class LineFeature{
    public:
        LineFeature() = default;
        ~LineFeature() = default;

        LineFeature& operator= (const LineFeature& line) = default;

        void push( LineFeature& line )
        {
            std::map<long, LineFeatureOb> line_obs = line.get_obs();
            for(auto &ob:line_obs)
            {
                obs.insert( ob );
            }
        }

        void push_set( LineFeature& line )
        {
            std::map<long, LineFeatureOb> line_obs = line.get_obs();
            for(auto &ob:line_obs)
            {
                obs.insert( ob );
            }
            cv::Mat des = line.get_descri();
            set_descri( des );
        }

        LineFeature3D get3D( const Eigen::Matrix4d& Twc = Eigen::Matrix4d::Identity() );

        void insert_ob(long id, const LineFeatureOb& ob)
        {
            obs.emplace(id, ob);
        }

        void insert_ob(long stereo_id, const Eigen::Vector4d& left_ob, const Eigen::Vector4d& right_ob)
        {
            LineFeatureOb ob_l(left_ob);
            LineFeatureOb ob_r(right_ob);
            obs.emplace(stereo_id*2, ob_l);
            obs.emplace(stereo_id*2+1, ob_r);
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
            // TODO update the parameter
            if(ids.count( obs.begin()->first ) < 1)
            {

            }
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

        std::tuple< bool, double > tri_two_plane( const std::map<long, Eigen::Matrix4d>& frame_map );

        // TODO yang 三角化
        std::tuple< bool, double > tri_yang( const std::map<long, Eigen::Matrix4d>& frame_map );

        std::map<long, LineFeatureOb> get_obs(){return obs;}

        // TODO 自动描述子求平均
        void set_descri(cv::Mat& des){ descri = des.clone(); }

        cv::Mat get_descri(){ return descri.clone(); }

        bool is_tri(){return tri;}

    private:
        std::map<long, LineFeatureOb> obs;  // < frame_id, observe >
        cv::Mat descri;

        double mini_tri_angle = 1.5;

        // TODO 生命周期过长的特征直接删除？
        int age = 1;

        bool tri = false;

        Eigen::Matrix<double, 6, 1> plucker_cam;  // plucker in first cam
        // descriptor
    };
}

#endif //LVO_FEATURE_HPP
