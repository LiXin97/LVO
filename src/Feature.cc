//
// Created by lixin04 on 20年1月14日.
//

#include "Feature.hpp"

namespace LVO{

    LineFeature3D LineFeature::get3D( const Eigen::Matrix4d& Twc  )
    {
        Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
        Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);


        Eigen::Vector3d nc = plucker_cam.head(3);
        Eigen::Vector3d vc = plucker_cam.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;


        auto id_obs_0 = obs.begin();

        Eigen::Vector3d ob00 = id_obs_0->second.get3dob0();
        Eigen::Vector3d ob01 = id_obs_0->second.get3dob1();
        Vector2d ln = ( ob00.cross(ob01) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Eigen::Vector3d p12 = Vector3d(ob00(0) + ln(0), ob00(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Eigen::Vector3d p22 = Vector3d(ob01(0) + ln(0), ob01(1) + ln(1), 1.0);
        Eigen::Vector3d cam = Vector3d( 0, 0, 0 );

        Eigen::Vector4d pi1 = pi_from_ppp(cam, ob00, p12);
        Eigen::Vector4d pi2 = pi_from_ppp(cam, ob01, p22);

        Eigen::Vector4d e1 = Lc * pi1;
        Eigen::Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

        double length = (e1-e2).norm();
        if(length > 10) {std::cerr << " length = " << length << std::endl;}

        //std::cout << e1 <<"\n\n";
        Eigen::Vector3d pts_1(e1(0),e1(1),e1(2));
        Eigen::Vector3d pts_2(e2(0),e2(1),e2(2));

        Eigen::Vector3d w_pts_1 =  Rwc * pts_1 + twc;
        Eigen::Vector3d w_pts_2 =  Rwc * pts_2 + twc;

        return LineFeature3D(w_pts_1, w_pts_2);
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

            plucker_cam = pipi_plk(p00, best_p);  // initial 3d line in camera frame


//            Eigen::Vector3d nc = line_c.head(3);
//            Eigen::Vector3d vc = line_c.tail(3);
//            Eigen::Vector3d nw = Rwc0 * nc + Utility::skewSymmetric(twc0) * Rwc0 * vc;
//            Eigen::Vector3d vw = Rwc0 * vc;
//
//            plucker.head(3) = nw;
//            plucker.tail(3) = vw;
            tri = true;

            return std::make_tuple(true, max_theta);
        }
    }
}