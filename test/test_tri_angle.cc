//
// Created by lixin04 on 20年1月15日.
//

#include "Feature.hpp"

int main()
{
    Eigen::Matrix4d Twc0 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Twc1 = Eigen::Matrix4d::Identity(); Twc1(0,3) = 1. / std::sqrt(3);
    Eigen::Matrix4d Tc1w = Eigen::Matrix4d::Identity(); Tc1w(0,3) = - 1. / std::sqrt(3);
    std::map<long, Eigen::Matrix4d> Twcs;
    Twcs.emplace( 0, Twc0 );
    Twcs.emplace( 1, Twc1 );
    {
        {

            Eigen::Vector4d left_ob( 0, 1, 0, -1 ), right_ob( -1./std::sqrt(3), -1, -1./std::sqrt(3), 1 );
            LVO::LineFeature linefeature;
            linefeature.insert_ob(0, left_ob, right_ob);
            bool tri_flag; double tri_plane_angle;
            std::tie(tri_flag, tri_plane_angle) = linefeature.tri_two_plane( Twcs );
//                if(tri_flag)
            std::cout << "tri_plane_angle = " << tri_plane_angle << std::endl;
        }
    }

    return 0;
}