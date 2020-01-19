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

        void retri_check( const std::map<long, Eigen::Matrix4d>& frame_map )
        {
            if(!is_tri())
                return;
            int num = 0;
            Eigen::Vector2d projection_error_avg(0., 0.);
            std::set<long> outlier_observeids;
            auto it = obs.begin();
            const long begin_i = it->first;

            Eigen::Matrix4d Twc0 = frame_map.find(begin_i)->second;

            Eigen::Vector3d twc0 = Twc0.block(0, 3, 3, 1);
            Eigen::Matrix3d Rwc0 = Twc0.block(0, 0, 3, 3);

            Eigen::Matrix<double, 6, 1> plk_w = plk_translation( plucker_cam, Rwc0, twc0 );
            it++;
            auto it_next = it;
            for(;it!=obs.end();it = it_next)
            {
                it_next++;

                auto obs_i = it->second.get4dob();

                Eigen::Matrix4d Twci = frame_map.find(it->first)->second;

                Eigen::Vector3d twci = Twci.block(0, 3, 3, 1);
                Eigen::Matrix3d Rwci = Twci.block(0, 0, 3, 3);

                Eigen::Matrix<double, 6, 1> line_c = plk_from_pose(plk_w, Rwci, twci);

                Eigen::Vector3d nc = line_c.head(3);
                double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
                double l_sqrtnorm = sqrt( l_norm );
                double l_trinorm = l_norm * l_sqrtnorm;

                double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
                double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
                Eigen::Vector2d residual;
                residual(0) = std::abs(e1/l_sqrtnorm);
                residual(1) = std::abs(e2/l_sqrtnorm);
                residual *= 460.;

//                std::cout << "residual =  " << residual.transpose() << std::endl;
                if(residual(0) > 1.5 || residual(1) > 1.5)
                {

//                    std::cerr << "bad " << residual.transpose() << std::endl;
                    obs.erase(it);
                }

                projection_error_avg += residual;
                num++;
            }

            projection_error_avg /= num;
            if(projection_error_avg(0) > 1.5 || projection_error_avg(1) > 1.5)
            {
//                std::cerr << "retri " << projection_error_avg.transpose() << std::endl;
                tri = false;
            }
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

        void remove_frame(long id,  const std::map<long, Eigen::Matrix4d>& frame_map)
        {
            // TODO test
            //
            if(id == obs.begin()->first)
            {
                Eigen::Matrix<double, 6, 1> plk_w;
                auto obs_it = obs.begin();
                {
                    auto iter = frame_map.find(obs_it->first);
                    Eigen::Matrix4d Twc = iter->second;

                    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
                    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
                    plk_w = plk_translation( plucker_cam, Rwc, twc );
                }

                {
                    obs_it++;
                    auto iter = frame_map.find(obs_it->first);
                    Eigen::Matrix4d Twc = iter->second;

                    Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
                    Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
                    plucker_cam = plk_from_pose(plk_w, Rwc, twc);
                }
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

        Eigen::Vector4d get_orth_w( const std::map<long, Eigen::Matrix4d>& frame_map )
        {
            auto iter = frame_map.find(obs.begin()->first);
            Eigen::Matrix4d Twc = iter->second;

            Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
            Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);

            Eigen::Matrix<double, 6, 1> plk_w = plk_translation( plucker_cam, Rwc, twc );

            Eigen::Vector4d orth = plk_to_orth(plk_w);
            return orth;
        }

        void set_orth_w( const Eigen::Vector4d& orth, const std::map<long, Eigen::Matrix4d>& frame_map )
        {
            Eigen::Matrix<double, 6, 1> plk_w = orth_to_plk(orth);


            auto iter = frame_map.find(obs.begin()->first);
            Eigen::Matrix4d Twc = iter->second;

            Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
            Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);

            Eigen::Matrix<double, 6, 1> line_c = plk_from_pose(plk_w, Rwc, twc);

            plucker_cam = line_c;
        }


        Eigen::Matrix<double, 6, 1> plk_translation( const Eigen::Matrix<double, 6, 1>& plk, const Eigen::Matrix3d& R, const Eigen::Vector3d& t )
        {
            Eigen::Vector3d n_src = plk.head(3);
            Eigen::Vector3d v_src = plk.tail(3);

            Eigen::Vector3d n_new = R * n_src + skew_symmetric(t) * R * v_src;
            Eigen::Vector3d v_new = R * v_src;

            Eigen::Matrix<double, 6, 1> plk_new;
            plk_new.head(3) = n_new;
            plk_new.tail(3) = v_new;
            return plk_new;
        }

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
