//
// Created by hyj on 17-12-8.
//
#include "include/line_uti/line_geometry.hpp"

Eigen::Vector2d get_spherical_from_unit(Eigen::Vector3d& unit_n)
{
    return Eigen::Vector2d(
            std::atan2(unit_n(1), unit_n(0)),
            std::atan2(unit_n(2), std::sqrt((unit_n(1)*unit_n(1) + unit_n(0)*unit_n(0)))));
}

Eigen::Vector3d get_unit_from_spherical(Eigen::Vector2d& spherical)
{
    return Eigen::Vector3d(
            std::cos(spherical(1)) * std::cos(spherical(0)),
            std::cos(spherical(1)) * std::sin(spherical(0)),
            std::sin(spherical(1)));
}

// plane_x 是法向量平面和归一化平面的交线。因为法向量平面永远会和归一化平面相交，所以可以这么做。
Vector6d get_lineincam_form_lixin4(double line_theta_obv, double line_inverse_distance, Eigen::Vector2d& n_c_theta)
{
    Eigen::Vector3d n_c = get_unit_from_spherical(n_c_theta);
    Eigen::Vector3d plane_z = n_c;
    Eigen::Vector3d plane_x(std::sin(n_c_theta(0)), -std::cos(n_c_theta(0)), 0);
    Eigen::Vector3d plane_y = plane_z.cross(plane_x); //plane_y.normalize();
//    std::cout <<"plane_y.norm() = " << plane_y.norm() << std::endl;

    Eigen::Matrix3d Rcplane;
    Rcplane.col(0) = plane_x;
    Rcplane.col(1) = plane_y;
    Rcplane.col(2) = plane_z;

//    std::cout << rot.transpose() << std::endl;
//    std::cout << rot.inverse() << std::endl;


    Eigen::Vector3d theta_line_ob(std::cos(line_theta_obv), std::sin(line_theta_obv), 0);
    Eigen::Vector3d theta_line_v = Rcplane * theta_line_ob;
//    theta_line_v.normalize();
    n_c /= line_inverse_distance;

    Vector6d line_cam;
    line_cam.head(3) = n_c;
    line_cam.tail(3) = theta_line_v;
//    std::cout << theta_line_v.norm() << std::endl;


    return line_cam;
}

Vector6d get_lineincam_form_lixin(double line_theta_obv, double line_inverse_distance, Eigen::Vector3d& ob0, Eigen::Vector3d& ob1)
{

    Eigen::Vector3d ob_v = ob1 - ob0;
    ob_v.normalize();

    Eigen::Matrix3d rot;
    Eigen::Vector3d n_ob = ob0.cross(ob1);
    n_ob.normalize();

    Eigen::Vector3d aix_x = ob_v;
    Eigen::Vector3d aix_y = n_ob.cross(ob_v); aix_y.normalize();
    Eigen::Vector3d aix_z = n_ob;
    rot.col(0) = aix_x;
    rot.col(1) = aix_y;
    rot.col(2) = aix_z;

//    std::cout << rot.transpose() << std::endl;
//    std::cout << rot.inverse() << std::endl;


    Eigen::Vector3d theta_line_ob(std::cos(line_theta_obv), std::sin(line_theta_obv), 0);
    Eigen::Vector3d theta_line_v = rot * theta_line_ob;
//    theta_line_v.normalize();
    n_ob /= line_inverse_distance;

    Vector6d line_cam;
    line_cam.head(3) = n_ob;
    line_cam.tail(3) = theta_line_v;
//    std::cout << theta_line_v.norm() << std::endl;


    return line_cam;
}

Vector4d line_to_orth(Vector6d line)
{
    Vector4d orth;
    Vector3d p = line.head(3);
    Vector3d v = line.tail(3);
    Vector3d n = p.cross(v);

    Vector3d u1 = n/n.norm();
    Vector3d u2 = v/v.norm();
    Vector3d u3 = u1.cross(u2);

    orth[0] = atan2( u2(2),u3(2) );
    orth[1] = asin( -u1(2) );
    orth[2] = atan2( u1(1),u1(0) );

    Vector2d w( n.norm(), v.norm() );
    w = w/w.norm();
    orth[3] = asin( w(1) );

    return orth;

}

Vector6d yang_to_plk(Eigen::Quaterniond& yang)
{
    Vector6d plk;

    double d = yang.norm();
    yang.normalize();


    Matrix3d R = yang.toRotationMatrix();

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);

    Vector3d n = d * u1;
    Vector3d v = u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    //Vector3d Q = -R.col(2) * d;
    //plk.head(3) = Q.cross(v);
    //plk.tail(3) = v;

    return plk;


}


Vector6d orth_to_line(Vector4d orth)
{
    Vector6d line;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R <<
      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1/w2;      // 原点到直线的距离

    line.head(3) = -R.col(2) * d;
    line.tail(3) = R.col(1);

    return line;


}

Vector4d plk_to_orth(Vector6d plk)
{
    Vector4d orth;
    Vector3d n = plk.head(3);
    Vector3d v = plk.tail(3);

    Vector3d u1 = n/n.norm();
    Vector3d u2 = v/v.norm();
    Vector3d u3 = u1.cross(u2);

    orth[0] = atan2( u2(2),u3(2) );
    orth[1] = asin( -u1(2) );
    orth[2] = atan2( u1(1),u1(0) );

    if(std::isnan(orth[1]))
    {
        int l;
        std::cerr << "TJLKDShfLKDSJFLKSJFlk" << std::endl;
        std::cin >> l;
    }

    Vector2d w( n.norm(), v.norm() );
    w = w/w.norm();
    orth[3] = asin( w(1) );

    return orth;

}


Vector6d orth_to_plk(Vector4d orth)
{
    Vector6d plk;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R <<
      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
      c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
          -s2,                  s1 * c2,                  c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1/w2;      // 原点到直线的距离

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    //Vector3d Q = -R.col(2) * d;
    //plk.head(3) = Q.cross(v);
    //plk.tail(3) = v;

    return plk;


}

/*
 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
 (a,b,c)^T = vector(AO) cross vector(BO)
 d = O.dot(cross(AO,BO))
 */
Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector4d pi;
    pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    pi /= pi.head(3).norm();
    return pi;
}

// 两平面相交得到直线的plucker 坐标
Vector6d pipi_plk( Vector4d pi1, Vector4d pi2){
    Vector6d plk;
    Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

    plk << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
    Eigen::Vector3d line_v = plk.tail(3);
    plk /= line_v.norm();
    return plk;
}

// 获取光心到直线的垂直点
Vector3d plucker_origin(Vector3d n, Vector3d v) {
    return v.cross(n) / v.dot(v);
}

// 反对称矩阵
Matrix3d skew_symmetric( Vector3d v ) {
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}



Vector3d point_to_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw , Vector3d pt_w ) {
    return Rcw * pt_w + tcw;
}

// 从相机坐标系到世界坐标系
Vector3d poit_from_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Vector3d pt_c ) {

    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return point_to_pose( Rwc, twc, pt_c );
}

Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw) {
    Vector6d line_c;

    Vector3d cp_w, dv_w;
    cp_w = line_w.head(3);
    dv_w = line_w.tail(3);

    Vector3d cp_c = point_to_pose( Rcw, tcw, cp_w );
    Vector3d dv_c = Rcw* dv_w;

    line_c.head(3) = cp_c;
    line_c.tail(3) = dv_c;

    return line_c;
}

Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw) {
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return line_to_pose( line_c, Rwc, twc );
}

// 世界坐标系到相机坐标系下
Vector6d plk_to_pose( Vector6d plk_w, const Eigen::Matrix3d& Rcw, Eigen::Vector3d tcw ) {
    Vector3d nw = plk_w.head(3);
    Vector3d vw = plk_w.tail(3);

    Vector3d nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
    Vector3d vc = Rcw * vw;

    Vector6d plk_c;
    plk_c.head(3) = nc;
    plk_c.tail(3) = vc;
    return plk_c;
}

Vector6d plk_from_pose( Vector6d plk_c, Eigen::Matrix3d Rcw, const Eigen::Vector3d& tcw ) {

    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return plk_to_pose( std::move(plk_c), Rwc, twc);
}