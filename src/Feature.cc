//
// Created by lixin04 on 20年1月14日.
//

#include "Feature.hpp"

namespace LVO{

    LineFeature3D LineFeature::get3D()
    {

    }

    std::tuple< bool, double > LineFeature::tri_two_plane( const std::map<long, Eigen::Matrix4d>& frame_map )
    {
        if(obs.size() < 2) return std::make_tuple(false, 0.);

        Eigen::Vector3d line_unit0;
        Eigen::Vector3d n_unit0;
        // 三角化线的方向向量
        {
            auto id_obs_0 = obs.begin();
            auto iter0 = frame_map.find(id_obs_0->first);
            Eigen::Matrix4d Twc0 = iter0->second;

            Eigen::Vector3d twc0 = Twc0.block(0, 3, 3, 1);
            Eigen::Matrix3d Rwc0 = Twc0.block(0, 0, 3, 3);

            Eigen::Vector3d ob00 = id_obs_0->second.get3dob0();
            Eigen::Vector3d ob01 = id_obs_0->second.get3dob1();

            Eigen::Vector4d p00 = pi_from_ppp(ob00, ob01, Vector3d(0, 0, 0));

            Eigen::Vector4d best_p;
            double max_theta = 0.;
            auto it = obs.begin();
            for( it++; it != obs.end(); ++it )
            {
                auto id_obs_i = it;
                auto iteri = frame_map.find(id_obs_i->first);
                Eigen::Matrix4d Twci = iteri->second;

                Eigen::Vector3d twci = Twci.block(0, 3, 3, 1);
                Eigen::Matrix3d Rwci = Twci.block(0, 0, 3, 3);

                Eigen::Vector3d obi0 = id_obs_i->second.get3dob0();
                Eigen::Vector3d obi1 = id_obs_i->second.get3dob1();

                Eigen::Vector3d t0i = Rwc0.transpose() * (twci - twc0);   // tij
                Eigen::Matrix3d R0i = Rwc0.transpose() * Rwci;          // Rij

                obi0 = R0i * obi0 + t0i;
                obi1 = R0i * obi1 + t0i;
                Eigen::Vector4d p0i = pi_from_ppp(obi0, obi1, t0i);

                Eigen::Vector3d p1n = p00.head(3);
                Eigen::Vector3d p2n = p0i.head(3);


                double theta = std::acos(p1n.dot(p2n)) / M_PI * 180.;
                if(theta > 90) theta = 180. - theta;

                if (theta > max_theta) {
                    max_theta = theta;
                    best_p = p0i;
                }
            }

            if( max_theta < mini_tri_angle ) return std::make_tuple(false, max_theta);

            Vector6d line_c = pipi_plk(p00, best_p);  // initial 3d line in camera frame


            Eigen::Vector3d nc = line_c.head(3);
            Eigen::Vector3d vc = line_c.tail(3);
            Eigen::Vector3d nw = Rwc0 * nc + Utility::skewSymmetric(twc0) * Rwc0 * vc;
            Eigen::Vector3d vw = Rwc0 * vc;

            plucker.head(3) = nw;
            plucker.tail(3) = vw;

            return std::make_tuple(true, max_theta);
        }
    }
}