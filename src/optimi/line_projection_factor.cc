#include "optimi/line_projection_factor.hpp"
#include "line_uti/line_geometry.hpp"
#include "optimi/line_parameterization.hpp"
#include "utility/utility.hpp"


template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}
Eigen::Matrix2d lineProjectionFactor::sqrt_info;

lineProjectionFactor::lineProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector4d line_orth( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3] );
    Vector6d line_w = orth_to_plk(line_orth);

    Eigen::Matrix3d Rwc(Qi);
    Eigen::Vector3d twc(Pi);
    Vector6d line_c = plk_from_pose(line_w, Rwc, twc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    double line_len = std::sqrt( std::pow( (obs_i(0) - obs_i(2) ), 2 ) + std::pow( (obs_i(1) - obs_i(3) ), 2 ) );
//    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
//    std::cout<< residual.transpose() <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwc.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwc.transpose() * (nw + skew_symmetric(dw) * twc) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwc.transpose() * dw);


            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[1]);

            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose()*skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
            jaco_Lw_orth.setZero();
            jaco_Lw_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lw_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lw_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lw_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,3,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }

    }

    if (jacobians && jacobians[0] && jacobians[1] && 0)
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 10> num_jacobian;
        for (int k = 0; k < 10; k++)
        {
            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector4d line_orth_ck( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);
            ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2) {           // line orth的前三个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l<< delta, 0.0;
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }
            else if (a == 3) {           // line orth的最后一个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l.setZero();
                delta_l[3]= delta.x();
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }

            Vector6d line_w_ck = orth_to_plk(line_orth_ck);

            Eigen::Matrix3d Rwc_ck(Qi_ck);
            Eigen::Vector3d twc_ck(Pi_ck);
            Vector6d line_c_ck = plk_from_pose(line_w_ck, Rwc_ck, twc_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_c_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );
            double l_trinorm_ck = l_norm_ck * l_sqrtnorm_ck;

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);
            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
    }


    return true;
}

Eigen::Matrix2d lineProjectionRightFactor::sqrt_info;

lineProjectionRightFactor::lineProjectionRightFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineProjectionRightFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pex(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qex(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3] );
    Vector6d line_w = orth_to_plk(line_orth);

    Eigen::Matrix3d Rwl(Qi);
    Eigen::Vector3d twl(Pi);
    Vector6d line_l = plk_from_pose(line_w, Rwl, twl);
    //std::cout << line_b.norm() <<"\n";
    Eigen::Matrix3d Rlr(Qex);
    Eigen::Vector3d tlr(Pex);
    Vector6d line_c = plk_from_pose(line_l, Rlr, tlr);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    double line_len = std::sqrt( std::pow( (obs_i(0) - obs_i(2) ), 2 ) + std::pow( (obs_i(1) - obs_i(3) ), 2 ) );
//    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
//    std::cout<< residual.transpose() <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc / line_len;
        //std::cout <<jaco_e_Lc<<"\n\n";
        //std::cout << "jacobian_calculator:" << std::endl;
        if (jacobians[0])
        {
            //std::cout <<"jacobian_pose_i"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Matrix6d invTlr;
            invTlr << Rlr.transpose(), -Rlr.transpose()*skew_symmetric(tlr),
                    Eigen::Matrix3d::Zero(),  Rlr.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwl.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwl.transpose() * (nw + skew_symmetric(dw) * twl) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwl.transpose() * dw);

            jaco_Lc_pose = invTlr * jaco_Lc_pose;
            //std::cout <<invTlr<<"\n"<<jaco_Lc_pose<<"\n\n";

            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {

            //std::cout <<"jacobian_ex_pose"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);

            Vector3d nb = line_l.head(3);
            Vector3d db = line_l.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = Rlr.transpose() * skew_symmetric(db);   // Lc_t
            jaco_Lc_ex.block(0,3,3,3) = skew_symmetric( Rlr.transpose() * (nb + skew_symmetric(db) * tlr) );  // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = skew_symmetric( Rlr.transpose() * db);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rwc = Rwl * Rlr;
            Eigen::Vector3d twc = Rwl * tlr + twl;
            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose()*skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
            jaco_Lw_orth.setZero();
            jaco_Lw_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lw_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lw_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lw_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,3,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }
    }

    if (jacobians && jacobians[0] && jacobians[1] && 0)
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 10> num_jacobian;
        for (int k = 0; k < 10; k++)
        {
            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector4d line_orth_ck( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);
            ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2) {           // line orth的前三个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l<< delta, 0.0;
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }
            else if (a == 3) {           // line orth的最后一个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l.setZero();
                delta_l[3]= delta.x();
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }

            Vector6d line_w_ck = orth_to_plk(line_orth_ck);

            Eigen::Matrix3d Rwc_ck(Qi_ck);
            Eigen::Vector3d twc_ck(Pi_ck);
            Vector6d line_c_ck = plk_from_pose(line_w_ck, Rwc_ck, twc_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_c_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );
            double l_trinorm_ck = l_norm_ck * l_sqrtnorm_ck;

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);
            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
    }


    return true;
}

Eigen::Matrix2d lineyangProjectionFactor::sqrt_info;
double lineyangProjectionFactor::sum_t;

lineyangProjectionFactor::lineyangProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineyangProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Quaterniond line_yang( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);


    Vector6d line_w = yang_to_plk(line_yang);
    double distance = line_yang.norm();
    line_yang.normalize();
    Eigen::Matrix3d line_R = line_yang.toRotationMatrix();

    Eigen::Matrix3d Rwc(Qi);
    Eigen::Vector3d twc(Pi);
    Vector6d line_c = plk_from_pose(line_w, Rwc, twc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    double line_len = std::sqrt( std::pow( (obs_i(0) - obs_i(2) ), 2 ) + std::pow( (obs_i(1) - obs_i(3) ), 2 ) );
//    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
//    std::cout<< residual.transpose() <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwc.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwc.transpose() * (nw + skew_symmetric(dw) * twc) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwc.transpose() * dw);


            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[1]);

            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose()*skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_Rd;
            Eigen::Matrix<double, 4, 4> jaco_Rd_yang;
            jaco_Lw_Rd.setZero();

            jaco_Lw_Rd.block(0,0,3,3) = distance * skew_symmetric(line_R * Eigen::Vector3d(1.,0,0));
            jaco_Lw_Rd.block(3,0,3,3) = skew_symmetric(line_R * Eigen::Vector3d(0,1.,0));

            jaco_Lw_Rd.block(0,3,3,1) = line_R * Eigen::Vector3d(1.,0,0);
            jaco_Lw_Rd.block(3,3,3,1).setZero();

            Eigen::Vector3d q_xyz = line_yang.vec();
            double q_w = line_yang.w();
            Eigen::Matrix3d Iden = Eigen::Matrix3d::Identity();

            jaco_Rd_yang.block(0,0,3,3) = 2./(distance * distance) * ( q_w * Iden - skew_symmetric(q_xyz) );
            jaco_Rd_yang.block(3,0,1,3) = q_xyz.transpose();
            jaco_Rd_yang.block(0,3,3,1) = -2./(distance * distance) * q_xyz;
            jaco_Rd_yang(3,3) = q_w;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_Rd * jaco_Rd_yang;
        }

    }

    if (jacobians && jacobians[0] && jacobians[1] && 1)
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 10> num_jacobian;
        for (int k = 0; k < 10; k++)
        {
            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector4d line_yang_ck( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);
            ceres::LocalParameterization *local_parameterization_line = new LineYangParameterization();

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2) {           // line orth的前三个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l<< delta, 0.0;
                local_parameterization_line->Plus(line_yang_ck.data(),delta_l.data(),line_new.data());
                line_yang_ck = line_new;
            }
            else if (a == 3) {           // line orth的最后一个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l.setZero();
                delta_l[3]= delta.x();
                local_parameterization_line->Plus(line_yang_ck.data(),delta_l.data(),line_new.data());
                line_yang_ck = line_new;
            }

//            Eigen::Quaterniond line_yang( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);

            Eigen::Quaterniond line_yang_ck_new(line_yang_ck(0), line_yang_ck(1), line_yang_ck(2), line_yang_ck(3));

            Vector6d line_w_ck = yang_to_plk(line_yang_ck_new);
//            double distance = line_yang.norm();
//            line_yang.normalize();
//            Eigen::Matrix3d line_R = line_yang.toRotationMatrix();
//
//            Vector6d line_w_ck = orth_to_plk(line_orth_ck);

            Eigen::Matrix3d Rwc_ck(Qi_ck);
            Eigen::Vector3d twc_ck(Pi_ck);
            Vector6d line_c_ck = plk_from_pose(line_w_ck, Rwc_ck, twc_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_c_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );
            double l_trinorm_ck = l_norm_ck * l_sqrtnorm_ck;

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);
            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;


        std::cout <<"----------------------------------"<<  std::endl;
    }


    return true;
}


Eigen::Matrix2d lineLixin2ProjectionFactor::sqrt_info;
double lineLixin2ProjectionFactor::sum_t;

lineLixin2ProjectionFactor::lineLixin2ProjectionFactor(const Eigen::Vector4d &_obs_i, const Eigen::Vector4d &_obs_j) : obs_i(_obs_i), obs_j(_obs_j)
{
};

bool lineLixin2ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double line_theta_ob_v = parameters[2][0];
    double line_distance = parameters[2][1];

    Eigen::Vector3d ob0(obs_i(0), obs_i(1), 1);
    Eigen::Vector3d ob1(obs_i(2), obs_i(3), 1);
    Vector6d line_cami = get_lineincam_form_lixin(line_theta_ob_v, line_distance, ob0, ob1);

    Eigen::Matrix3d Rwci = Qi.toRotationMatrix();
    Eigen::Vector3d twci(Pi);
    Vector6d line_w = plk_to_pose(line_cami, Rwci, twci);

    Eigen::Matrix3d Rwcj = Qj.toRotationMatrix();
    Eigen::Vector3d twcj(Pj);
    Vector6d line_camj = plk_from_pose(line_w, Rwcj, twcj);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_camj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_j(0) * nc(0) + obs_j(1) * nc(1) + nc(2);
    double e2 = obs_j(2) * nc(0) + obs_j(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    residual = sqrt_info * residual;
//    std::cout << residual.transpose() << std::endl;

    if (jacobians)
    {
        Eigen::Vector3d ob_v = ob1 - ob0;
        ob_v.normalize();

        Eigen::Matrix3d rot_cam_obp;
        Eigen::Vector3d n_ob = ob0.cross(ob1);
        n_ob.normalize();

        Eigen::Vector3d aix_x = ob_v;
        Eigen::Vector3d aix_y = n_ob.cross(ob_v); aix_y.normalize();
        Eigen::Vector3d aix_z = n_ob;
        rot_cam_obp.col(0) = aix_x;
        rot_cam_obp.col(1) = aix_y;
        rot_cam_obp.col(2) = aix_z;

        Eigen::Vector3d jaco_theta_line_ob(-std::sin(line_theta_ob_v), std::cos(line_theta_ob_v), 0);


        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_j(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_j(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_j(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_j(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_cami.head(3);
            Vector3d dbi = line_cami.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwci * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwci * skew_symmetric( nbi) - skew_symmetric(twci) * Rwci * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwci * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);


            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwcj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwcj.transpose() * (nw + skew_symmetric(dw) * twcj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwcj.transpose() * dw);

            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jacobian_linelixin(jacobians[2]);

            Eigen::Matrix3d Rcjci = Rwcj.transpose() * Rwci;
            Vector3d tcjci =  ( Rwcj.transpose() * ( twci - twcj) );

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_cami.head(3);
            Vector3d vci = line_cami.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 2> jaco_Lc_lixin;
            jaco_Lc_lixin.setZero();
            jaco_Lc_lixin.block(0,1,3,1) = - n_ob / (line_distance * line_distance);
            jaco_Lc_lixin.block(3,0,3,1) = rot_cam_obp * jaco_theta_line_ob;


            jacobian_linelixin = jaco_e_Lc * Tcjci * jaco_Lc_lixin;
        }

    }

    if (0 && jacobians[0] && jacobians[1] && jacobians[2])
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>(jacobians[2]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 14> num_jacobian;
        for (int k = 0; k < 14; k++)
        {

            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

            double line_theta_ob_v_ck = parameters[2][0];
            double line_distance_ck = parameters[2][1];

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2)
                Pj_ck += delta;
            else if (a == 3)
                Qj_ck = Qj_ck * deltaQ(delta);
            else if ( k == 12 )
                line_theta_ob_v_ck += eps;
            else if( k == 13 )
                line_distance_ck += eps;




            Vector6d line_cami_ck = get_lineincam_form_lixin(line_theta_ob_v_ck, line_distance_ck, ob0, ob1);

            Eigen::Matrix3d Rwci_ck = Qi_ck.toRotationMatrix();
            Eigen::Vector3d twci_ck(Pi_ck);
            Vector6d line_w_ck = plk_to_pose(line_cami_ck, Rwci_ck, twci_ck);

            Eigen::Matrix3d Rwcj_ck = Qj_ck.toRotationMatrix();
            Eigen::Vector3d twcj_ck(Pj_ck);
            Vector6d line_camj_ck = plk_from_pose(line_w_ck, Rwcj_ck, twcj_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_camj_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );

            double e1_ck = obs_j(0) * nc_ck(0) + obs_j(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_j(2) * nc_ck(0) + obs_j(3) * nc_ck(1) + nc_ck(2);

            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n"<< num_jacobian.block(0,0,2,6) <<"\n"<< std::endl;
        std::cout << num_jacobian.block(0,6,2,6) <<"\n"<< std::endl;
        std::cout << num_jacobian.block(0,12,2,2) <<"\n"<< std::endl;


        std::cout <<"---------------------------------"<< std::endl;
    }


    return true;
}


Eigen::Matrix2d lineLixin4ProjectionFactor::sqrt_info;
double lineLixin4ProjectionFactor::sum_t;

lineLixin4ProjectionFactor::lineLixin4ProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineLixin4ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double line_theta_ob_v = parameters[2][0];
    double line_distance = parameters[2][1];
    Eigen::Vector2d line_n_c_theta(parameters[2][2], parameters[2][3]);

    Vector6d line_cami = get_lineincam_form_lixin4(line_theta_ob_v, line_distance, line_n_c_theta);

    Eigen::Matrix3d Rwci = Qi.toRotationMatrix();
    Eigen::Vector3d twci(Pi);
    Vector6d line_w = plk_to_pose(line_cami, Rwci, twci);

    Eigen::Matrix3d Rwcj = Qj.toRotationMatrix();
    Eigen::Vector3d twcj(Pj);
    Vector6d line_camj = plk_from_pose(line_w, Rwcj, twcj);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_camj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    residual = sqrt_info * residual;
//    std::cout << residual.transpose() << std::endl;

    if (jacobians)
    {

        Eigen::Vector3d n_c = get_unit_from_spherical(line_n_c_theta);
        Eigen::Vector3d plane_z = n_c;
        Eigen::Vector3d plane_x(n_c(1), -n_c(0), 0); plane_x.normalize();
        Eigen::Vector3d plane_y = plane_z.cross(plane_x); plane_z.normalize();

        Eigen::Matrix3d rot_cam_obp;
        rot_cam_obp.col(0) = plane_x;
        rot_cam_obp.col(1) = plane_y;
        rot_cam_obp.col(2) = plane_z;

        Eigen::Vector3d jaco_theta_line_ob(-std::sin(line_theta_ob_v), std::cos(line_theta_ob_v), 0);


        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();

            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_cami.head(3);
            Vector3d dbi = line_cami.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwci * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwci * skew_symmetric( nbi) - skew_symmetric(twci) * Rwci * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwci * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            jacobian_pose_j.setZero();


            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwcj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwcj.transpose() * (nw + skew_symmetric(dw) * twcj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwcj.transpose() * dw);

            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_linelixin(jacobians[2]);
            jacobian_linelixin.setZero();

            Eigen::Matrix3d Rcjci = Rwcj.transpose() * Rwci;
            Vector3d tcjci =  ( Rwcj.transpose() * ( twci - twcj) );

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            double s0 = ceres::sin(line_n_c_theta(0));
            double c0 = ceres::cos(line_n_c_theta(0));
            double s1 = ceres::sin(line_n_c_theta(1));
            double c1 = ceres::cos(line_n_c_theta(1));

            double x = std::cos(line_theta_ob_v);
            double y = std::sin(line_theta_ob_v);

            Eigen::Matrix<double, 3, 2> jaco_nc_angle;
            jaco_nc_angle <<
            -c1 * s0, -s1 * c0,
            c1 * c0, -s1 * s0,
            0, c1;


            Eigen::Matrix<double, 3, 2> jaco_vc_angle;
            jaco_vc_angle <<
            x*c0 - y*s1*s0, y*c1,
            x*s0 + y*s1*s0, y*c1,
            2.*y*(c1*c0*s0 - c1*s0*c0), s1*y*(s0*s0 + c0*c0);

//            x*c1*c0 - y*s1*c1*s0 , -x*s1*s0 + y*c1*c1*c0 - y*s1*s1*c0,
//            x*c1*s0 + y*s1*c1*c0, x*s1*c0 + y*c1*c1*s0 - y*s1*s1*s0,
//            -2.*y*c1*c1*s0*c0 + 2.*y*c1*c1*c0*s0, 2.*y*c1*s1*s0*s0 + 2.*y*c1*s1*c0*c0;

            Eigen::Matrix<double, 6, 4> jaco_Lc_lixin;
            jaco_Lc_lixin.setZero();
            jaco_Lc_lixin.block(0,1,3,1) = - n_c / (line_distance * line_distance);
            jaco_Lc_lixin.block(3,0,3,1) = rot_cam_obp * jaco_theta_line_ob;
            jaco_Lc_lixin.block(0,2,3,2) = jaco_nc_angle / line_distance;
            jaco_Lc_lixin.block(3,2,3,2) = jaco_vc_angle;


            jacobian_linelixin = jaco_e_Lc * Tcjci * jaco_Lc_lixin;
        }

    }

    if (0 && jacobians && jacobians[0] && jacobians[1] && jacobians[2])
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[2]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 16> num_jacobian;
        for (int k = 0; k < 16; k++)
        {

            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

            double line_theta_ob_v_ck = parameters[2][0];
            double line_distance_ck = parameters[2][1];
            Eigen::Vector2d line_n_c_theta_ck(parameters[2][2], parameters[2][3]);

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2)
                Pj_ck += delta;
            else if (a == 3)
                Qj_ck = Qj_ck * deltaQ(delta);
            else if ( k == 12 )
                line_theta_ob_v_ck += eps;
            else if( k == 13 )
                line_distance_ck += eps;
            else if( k == 14 )
                line_n_c_theta_ck(0) += eps;
            else if( k == 15 )
                line_n_c_theta_ck(1) += eps;




            Vector6d line_cami_ck = get_lineincam_form_lixin4(line_theta_ob_v_ck, line_distance_ck, line_n_c_theta_ck);


            Eigen::Matrix3d Rwci_ck = Qi_ck.toRotationMatrix();
            Eigen::Vector3d twci_ck(Pi_ck);
            Vector6d line_w_ck = plk_to_pose(line_cami_ck, Rwci_ck, twci_ck);

            Eigen::Matrix3d Rwcj_ck = Qj_ck.toRotationMatrix();
            Eigen::Vector3d twcj_ck(Pj_ck);
            Vector6d line_camj_ck = plk_from_pose(line_w_ck, Rwcj_ck, twcj_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_camj_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);

            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n"<< num_jacobian.block(0,0,2,6) <<"\n"<< std::endl;
        std::cout << num_jacobian.block(0,6,2,6) <<"\n"<< std::endl;
        std::cout << num_jacobian.block(0,12,2,4) <<"\n"<< std::endl;
        std::cout <<"---------------------------------"<< std::endl;


        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_linelixin(jacobians[2]);
        jacobian_linelixin.block(0,2,2,2) = num_jacobian.block(0,14,2,2);
    }

    if (1 && jacobians  && jacobians[2])
    {
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 2> num_jacobian;
        for (int k = 0; k < 2; k++)
        {

            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

            double line_theta_ob_v_ck = parameters[2][0];
            double line_distance_ck = parameters[2][1];
            Eigen::Vector2d line_n_c_theta_ck(parameters[2][2], parameters[2][3]);

            if ( k == 0)
                line_n_c_theta_ck(0) += eps;
            else if( k == 1 )
                line_n_c_theta_ck(1) += eps;




            Vector6d line_cami_ck = get_lineincam_form_lixin4(line_theta_ob_v_ck, line_distance_ck, line_n_c_theta_ck);


            Eigen::Matrix3d Rwci_ck = Qi_ck.toRotationMatrix();
            Eigen::Vector3d twci_ck(Pi_ck);
            Vector6d line_w_ck = plk_to_pose(line_cami_ck, Rwci_ck, twci_ck);

            Eigen::Matrix3d Rwcj_ck = Qj_ck.toRotationMatrix();
            Eigen::Vector3d twcj_ck(Pj_ck);
            Vector6d line_camj_ck = plk_from_pose(line_w_ck, Rwcj_ck, twcj_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_camj_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);

            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }


        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_linelixin(jacobians[2]);
        jacobian_linelixin.block(0,2,2,2) = num_jacobian;
    }


    return true;
}



Eigen::Matrix2d linezhaoliangProjectionFactor::sqrt_info;
double linezhaoliangProjectionFactor::sum_t;

linezhaoliangProjectionFactor::linezhaoliangProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool linezhaoliangProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d Pt(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qt(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);


    Eigen::Vector2d nic_angle(parameters[3][0], parameters[3][1]);
    Eigen::Vector2d njc_angle(parameters[3][2], parameters[3][3]);

    Eigen::Vector3d nic(
            ceres::cos(nic_angle(1)) * ceres::cos(nic_angle(0)),
            ceres::cos(nic_angle(1)) * ceres::sin(nic_angle(0)),
            ceres::sin(nic_angle(1)) );

    Eigen::Vector3d njc(
            ceres::cos(njc_angle(1)) * ceres::cos(njc_angle(0)),
            ceres::cos(njc_angle(1)) * ceres::sin(njc_angle(0)),
            ceres::sin(njc_angle(1)) );

    Eigen::Vector3d niw = Qi * nic;
    Eigen::Vector3d njw = Qj * njc;


    Eigen::Vector3d ntw = (Pj - Pt).dot(njw) * niw - (Pi - Pt).dot(niw) * njw;

    Eigen::Vector3d ntc = Qt.inverse() * ntw;

    double l_norm = ntc(0) * ntc(0) + ntc(1) * ntc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * ntc(0) + obs_i(1) * ntc(1) + ntc(2);
    double e2 = obs_i(2) * ntc(0) + obs_i(3) * ntc(1) + ntc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    residual = sqrt_info * residual;
//    std::cout<< residual.transpose() <<std::endl;

    if (jacobians)
    {
        Eigen::Matrix3d Rwci = Qi.toRotationMatrix();
        Eigen::Matrix3d Rwcj = Qj.toRotationMatrix();
        Eigen::Matrix3d Rwct = Qt.toRotationMatrix();
        Eigen::Vector3d twci = Pi;
        Eigen::Vector3d twcj = Pj;
        Eigen::Vector3d twct = Pt;

        Eigen::Matrix3d Iden = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 3> jaco_e_ntc(2, 3);
        jaco_e_ntc << (obs_i(0)/l_sqrtnorm - ntc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - ntc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - ntc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - ntc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_ntc = sqrt_info * jaco_e_ntc;

        Eigen::Matrix<double, 3, 3> jaco_ntc_ntw = Rwct.transpose();

        Eigen::Matrix<double, 3, 3> jaco_ntc_Rwct = Utility::skewSymmetric(Rwct.transpose() * ntw);

        Eigen::Matrix<double, 3, 2> jaco_nic_angle;
        jaco_nic_angle << -ceres::cos(nic_angle(1)) * ceres::sin(nic_angle(0)), -ceres::sin(nic_angle(1)) * ceres::cos(nic_angle(0)),
                ceres::cos(nic_angle(1)) * ceres::cos(nic_angle(0)), -ceres::sin(nic_angle(1)) * ceres::sin(nic_angle(0)),
                0, ceres::cos(nic_angle(1));

        Eigen::Matrix<double, 3, 2> jaco_njc_angle;
        jaco_njc_angle <<-ceres::cos(njc_angle(1)) * ceres::sin(njc_angle(0)), -ceres::sin(njc_angle(1)) * ceres::cos(njc_angle(0)),
                ceres::cos(njc_angle(1)) * ceres::cos(njc_angle(0)), -ceres::sin(njc_angle(1)) * ceres::sin(njc_angle(0)),
                0, ceres::cos(njc_angle(1));

        Eigen::Matrix<double, 3, 3> jaco_niw_nic = Rwci;
        Eigen::Matrix<double, 3, 3> jaco_njw_njc = Rwcj;

        Eigen::Matrix<double, 3, 3> jaco_niw_Rwci = -Rwci * Utility::skewSymmetric(nic);
        Eigen::Matrix<double, 3, 3> jaco_njw_Rwcj = -Rwcj * Utility::skewSymmetric(njc);

        Eigen::Matrix<double, 3, 3> jaco_ntw_niw_sub;
        jaco_ntw_niw_sub.row(0) = (twci - twct).transpose() * njw(0);
        jaco_ntw_niw_sub.row(1) = (twci - twct).transpose() * njw(1);
        jaco_ntw_niw_sub.row(2) = (twci - twct).transpose() * njw(2);

        Eigen::Matrix<double, 3, 3> jaco_ntw_niw = (twcj - twct).dot(njw) * Iden - jaco_ntw_niw_sub;


        Eigen::Matrix<double, 3, 3> jaco_ntw_njw_sub;
        jaco_ntw_njw_sub.row(0) = (twcj - twct).transpose() * niw(0);
        jaco_ntw_njw_sub.row(1) = (twcj - twct).transpose() * niw(1);
        jaco_ntw_njw_sub.row(2) = (twcj - twct).transpose() * niw(2);

        Eigen::Matrix<double, 3, 3> jaco_ntw_njw = jaco_ntw_njw_sub - (twci - twct).dot(niw) * Iden ;

        Eigen::Matrix<double, 3, 3> jaco_ntw_Rwci = jaco_ntw_niw * jaco_niw_Rwci;
        Eigen::Matrix<double, 3, 3> jaco_ntw_Rwcj = jaco_ntw_njw * jaco_njw_Rwcj;

        Eigen::Matrix<double, 3, 3> jaco_ntw_twci;
        jaco_ntw_twci.row(0) = -(niw).transpose() * njw(0);
        jaco_ntw_twci.row(1) = -(niw).transpose() * njw(1);
        jaco_ntw_twci.row(2) = -(niw).transpose() * njw(2);

        Eigen::Matrix<double, 3, 3> jaco_ntw_twcj;
        jaco_ntw_twcj.row(0) = (njw).transpose() * niw(0);
        jaco_ntw_twcj.row(1) = (njw).transpose() * niw(1);
        jaco_ntw_twcj.row(2) = (njw).transpose() * niw(2);


        Eigen::Matrix<double, 3, 3> jaco_ntw_twct;
        jaco_ntw_twct.row(0) = -(njw).transpose() * niw(0) + (niw).transpose() * njw(0);
        jaco_ntw_twct.row(1) = -(njw).transpose() * niw(1) + (niw).transpose() * njw(1);
        jaco_ntw_twct.row(2) = -(njw).transpose() * niw(2) + (niw).transpose() * njw(2);

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();
            jacobian_pose_i.block(0,0,2,3) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_twci;
            jacobian_pose_i.block(0,3,2,3) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_Rwci;
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            jacobian_pose_j.setZero();
            jacobian_pose_j.block(0,0,2,3) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_twcj;
            jacobian_pose_j.block(0,3,2,3) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_Rwcj;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_t(jacobians[2]);
            jacobian_pose_t.setZero();
            jacobian_pose_t.block(0,0,2,3) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_twct;
            jacobian_pose_t.block(0,3,2,3) = jaco_e_ntc * jaco_ntc_Rwct;
        }

        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_linezhaoliang(jacobians[3]);
            jacobian_linezhaoliang.setZero();
            jacobian_linezhaoliang.block(0,0,2,2) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_niw * jaco_niw_nic * jaco_nic_angle;
            jacobian_linezhaoliang.block(0,2,2,2) = jaco_e_ntc * jaco_ntc_ntw * jaco_ntw_njw * jaco_njw_njc * jaco_njc_angle;
        }

    }

    if(0 && jacobians && jacobians[0] && jacobians[1])
    {
        // check jacobian
        std::cout <<"ana_jacobian:\n";
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 22> num_jacobian;
        for (int k = 0; k < 22; k++)
        {
            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

            Eigen::Vector3d Pt_ck(parameters[2][0], parameters[2][1], parameters[2][2]);
            Eigen::Quaterniond Qt_ck(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

            Eigen::Vector2d nic_angle_ck(parameters[3][0], parameters[3][1]);
            Eigen::Vector2d njc_angle_ck(parameters[3][2], parameters[3][3]);

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2)
                Pj_ck += delta;
            else if (a == 3)
                Qj_ck = Qj_ck * deltaQ(delta);
            else if (a == 4)
                Pt_ck += delta;
            else if (a == 5)
                Qt_ck = Qt_ck * deltaQ(delta);
            else if( k == 18 )
                nic_angle_ck(0) += eps;
            else if( k == 19 )
                nic_angle_ck(1) += eps;
            else if( k == 20 )
                njc_angle_ck(0) += eps;
            else if( k == 21 )
                njc_angle_ck(1) += eps;


            Eigen::Vector3d nic_ck = get_unit_from_spherical(nic_angle_ck);
            Eigen::Vector3d njc_ck = get_unit_from_spherical(njc_angle_ck);


            Eigen::Vector3d niw_ck = Qi_ck * nic_ck;
            Eigen::Vector3d njw_ck = Qj_ck * njc_ck;


            Eigen::Vector3d ntw_ck = (Pj_ck - Pt_ck).dot(njw_ck) * niw_ck - (Pi_ck - Pt_ck).dot(niw_ck) * njw_ck;

            Eigen::Vector3d ntc_ck = Qt_ck.inverse() * ntw_ck;

            double l_norm_ck = ntc_ck(0) * ntc_ck(0) + ntc_ck(1) * ntc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );

            double e1_ck = obs_i(0) * ntc_ck(0) + obs_i(1) * ntc_ck(1) + ntc_ck(2);
            double e2_ck = obs_i(2) * ntc_ck(0) + obs_i(3) * ntc_ck(1) + ntc_ck(2);
            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

//        std::cout << tmp_residual.transpose() << std::endl;
//        std::cout << residual.transpose() << std::endl;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;
            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n";

        std::cout << num_jacobian.block(0,0,2,6) << std::endl;
        std::cout << num_jacobian.block(0,6,2,6) << std::endl;
        std::cout << num_jacobian.block(0,12,2,6) << std::endl;
        std::cout << num_jacobian.block(0,18,2,4) << std::endl;

        std::cout << " ----  " << std::endl;
    }
    return true;
}




Eigen::Matrix2d lineQuaProjectionFactor::sqrt_info;
double lineQuaProjectionFactor::sum_t;

lineQuaProjectionFactor::lineQuaProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Tbc
  parameters[2]:  line_orth
*/
bool lineQuaProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Quaterniond line_Q(parameters[1][4], parameters[1][1], parameters[1][2], parameters[1][3]);
    double phi = parameters[1][0];

    Eigen::Matrix3d line_R = line_Q.toRotationMatrix();

    double w1 = cos(phi);
    double w2 = sin(phi);

    Vector3d u1 = line_R.col(0);
    Vector3d u2 = line_R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    Vector6d line_w ;
    line_w.head(3) = n;
    line_w.tail(3) = v;


    Eigen::Matrix3d Rwc(Qi);
    Eigen::Vector3d twc(Pi);
    Vector6d line_c = plk_from_pose(line_w, Rwc, twc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    residual = sqrt_info * residual;
//    std::cout<< residual.transpose() <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwc.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwc.transpose() * (nw + skew_symmetric(dw) * twc) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwc.transpose() * dw);

            //std::cout <<invTbc<<"\n"<<jaco_Lc_pose<<"\n\n";

            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jacobian_lineOrth(jacobians[1]);

            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose()*skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 5> jaco_Lw_orth;
            jaco_Lw_orth.setZero();

            jaco_Lw_orth.block(0,1,3,3) = - w(0) * line_R * skew_symmetric(Eigen::Vector3d(1.,0,0));
            jaco_Lw_orth.block(3,1,3,3) = - w(1) * line_R * skew_symmetric(Eigen::Vector3d(0,1.,0));

            jaco_Lw_orth.block(0,0,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,0,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }

    }

    if(0 && jacobians && jacobians[0] && jacobians[1])
    {
        // check jacobian
        std::cout <<"ana_jacobian:\n";
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 10> num_jacobian;
        for (int k = 0; k < 10; k++)
        {
            Eigen::Vector3d Pc(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qc(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Quaterniond line_Q_ck(parameters[1][4], parameters[1][1], parameters[1][2], parameters[1][3]);
            double phi_ck = parameters[1][0];


            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pc += delta;
            else if (a == 1)
                Qc = Qi * deltaQ(delta);
            else if (a == 2)
                line_Q_ck = line_Q_ck * deltaQ(delta);
            else if (a == 3)
            {
                double w1_ck_ = cos(phi_ck);
                double w2_ck_ = sin(phi_ck);
                Eigen::Matrix2d W_ck;
                W_ck << w1_ck_, -w2_ck_, w2_ck_, w1_ck_;
                Eigen::Matrix2d delta_W;
                delta_W << cos(eps), -sin(eps),sin(eps), cos(eps);
                W_ck = W_ck * delta_W;
                phi_ck = asin( W_ck(1,0) );
            }


            Eigen::Matrix3d line_R_ck = line_Q_ck.toRotationMatrix();

            double w1_ck = cos(phi_ck);
            double w2_ck = sin(phi_ck);

            Vector3d u1_ck = line_R_ck.col(0);
            Vector3d u2_ck = line_R_ck.col(1);

            Vector3d n_ck = w1_ck * u1_ck;
            Vector3d v_ck = w2_ck * u2_ck;

            Vector6d line_w_ck ;
            line_w_ck.head(3) = n_ck;
            line_w_ck.tail(3) = v_ck;

            Eigen::Vector3d twc_ck(Pc);
            Eigen::Matrix3d Rwc_ck(Qc);
            Vector6d line_c_ck = plk_from_pose(line_w_ck, Rwc_ck, twc_ck);


            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_c_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt( l_norm_ck );

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);
            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck/l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck/l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

//        std::cout << tmp_residual.transpose() << std::endl;
//        std::cout << residual.transpose() << std::endl;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;
            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout <<"num_jacobian:\n";

        std::cout << num_jacobian.leftCols(6) << std::endl;
        std::cout << num_jacobian.rightCols(4) << std::endl;

        std::cout << " ----  " << std::endl;
    }
    return true;
}


//////////////////////////////////////////////////
Eigen::Matrix2d lineProjectionFactor_in_no_camera::sqrt_info;
lineProjectionFactor_in_no_camera::lineProjectionFactor_in_no_camera(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineProjectionFactor_in_no_camera::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
    Vector6d line_cami = orth_to_plk(line_orth);

    Eigen::Matrix3d Rwci = Qi.toRotationMatrix();
    Eigen::Vector3d twci(Pi);
    Vector6d line_w = plk_to_pose(line_cami, Rwci, twci);

    Eigen::Matrix3d Rwcj = Qj.toRotationMatrix();
    Eigen::Vector3d twcj(Pj);
    Vector6d line_camj = plk_from_pose(line_w, Rwcj, twcj);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_camj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    residual = sqrt_info * residual;
//    std::cout << residual.transpose() << std::endl;

    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

/*
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Matrix6d invTwbj;
            invTwbj << Rwbj.transpose(), -Rwbj.transpose()*skew_symmetric(twbj),
                 en::Matrix3d::Zero(),  Rwbj.transpose();
*/

            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_cami.head(3);
            Vector3d dbi = line_cami.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwci * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwci * skew_symmetric( nbi) - skew_symmetric(twci) * Rwci * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwci * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);


            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwcj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwcj.transpose() * (nw + skew_symmetric(dw) * twcj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwcj.transpose() * dw);

            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rcjci = Rwcj.transpose() * Rwci;
            Vector3d tcjci =  ( Rwcj.transpose() * ( twci - twcj) );

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_cami.head(3);
            Vector3d vci = line_cami.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lc_orth;
            jaco_Lc_orth.setZero();
            jaco_Lc_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lc_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lc_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lc_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_orth;
        }

    }

    if (0 && jacobians[0] && jacobians[1] && jacobians[2])
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[2]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 16> num_jacobian;
        for (int k = 0; k < 16; k++) {

            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);


            Eigen::Vector4d line_orth_ck( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);

            ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2)
                Pj_ck += delta;
            else if (a == 3)
                Qj_ck = Qj_ck * deltaQ(delta);
            else if (a == 4) {           // line orth的前三个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l<< delta, 0.0;
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }
            else if (a == 5) {           // line orth的最后一个元素
                Eigen::Vector4d line_new;
                Eigen::Vector4d delta_l;
                delta_l.setZero();
                delta_l[3]= delta.x();
                local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                line_orth_ck = line_new;
            }

            Vector6d line_cami_ck = orth_to_plk(line_orth_ck);

            Eigen::Matrix3d Rwci_ck = Qi_ck.toRotationMatrix();
            Eigen::Vector3d twci_ck(Pi_ck);
            Vector6d line_w_ck = plk_to_pose(line_cami_ck, Rwci_ck, twci_ck);

            Eigen::Matrix3d Rwcj_ck = Qj_ck.toRotationMatrix();
            Eigen::Vector3d twcj_ck(Pj_ck);
            Vector6d line_camj_ck = plk_from_pose(line_w_ck, Rwcj_ck, twcj_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_camj_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt(l_norm_ck);

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);

            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck / l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck / l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout << "num_jacobian:\n"
        << num_jacobian.block(0,0,2,6) << "\n"
        << num_jacobian.block(0,6,2,6) << "\n"
        << num_jacobian.block(0,12,2,4) << "\n"
        << std::endl;



        std::cout << "----------------------------" << std::endl;
    }
/*
    // check jacobian
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    if(jacobians[3])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 22> num_jacobian;// 3 * 6 + 4
    for (int k = 0; k < 22; k++)
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * deltaQ(delta);
        else if (a == 6) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 7) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_ci = orth_to_plk(line_orth);
        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

        Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
        Eigen::Vector3d twbi(Pi);
        Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

        Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
        Eigen::Vector3d twbj(Pj);
        Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

        Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_cj.head(3);

        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}

//////////////////////////////////////////////////
Eigen::Matrix2d lineProjectionFactor_in_no_camera_qua::sqrt_info;
lineProjectionFactor_in_no_camera_qua::lineProjectionFactor_in_no_camera_qua(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

bool lineProjectionFactor_in_no_camera_qua::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Quaterniond line_qua( parameters[2][4],parameters[2][1],parameters[2][2],parameters[2][3]);
    double phi = parameters[2][0];

    Eigen::Matrix3d line_R = line_qua.toRotationMatrix();

    double w1 = cos(phi);
    double w2 = sin(phi);

    Vector3d u1 = line_R.col(0);
    Vector3d u2 = line_R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    Vector6d line_cami ;
    line_cami.head(3) = n;
    line_cami.tail(3) = v;

    Eigen::Matrix3d Rwci = Qi.toRotationMatrix();
    Eigen::Vector3d twci(Pi);
    Vector6d line_w = plk_to_pose(line_cami, Rwci, twci);

    Eigen::Matrix3d Rwcj = Qj.toRotationMatrix();
    Eigen::Vector3d twcj(Pj);
    Vector6d line_camj = plk_from_pose(line_w, Rwcj, twcj);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_camj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    residual = sqrt_info * residual;
//    std::cout << residual.transpose() << std::endl;

    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

/*
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Matrix6d invTwbj;
            invTwbj << Rwbj.transpose(), -Rwbj.transpose()*skew_symmetric(twbj),
                 en::Matrix3d::Zero(),  Rwbj.transpose();
*/

            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_cami.head(3);
            Vector3d dbi = line_cami.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwci * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwci * skew_symmetric( nbi) - skew_symmetric(twci) * Rwci * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwci * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);


            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwcj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwcj.transpose() * (nw + skew_symmetric(dw) * twcj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwcj.transpose() * dw);

            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rcjci = Rwcj.transpose() * Rwci;
            Vector3d tcjci =  ( Rwcj.transpose() * ( twci - twcj) );

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_cami.head(3);
            Vector3d vci = line_cami.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 5> jaco_Lc_Qua;
            jaco_Lc_Qua.setZero();

            jaco_Lc_Qua.block(0,1,3,3) = - w(0) * line_R * skew_symmetric(Eigen::Vector3d(1.,0,0));
            jaco_Lc_Qua.block(3,1,3,3) = - w(1) * line_R * skew_symmetric(Eigen::Vector3d(0,1.,0));

            jaco_Lc_Qua.block(0,0,3,1) = -w(1) * u1;
            jaco_Lc_Qua.block(3,0,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_Qua;
        }

    }

    if (0 && jacobians[0] && jacobians[1] && jacobians[2])
    {
        // check jacobian
        std::cout << "ana = " << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
                  << std::endl;
        std::cout << Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>>(jacobians[2]) << std::endl
                  << std::endl;
        const double eps = 1e-6;
        Eigen::Matrix<double, 2, 16> num_jacobian;
        for (int k = 0; k < 16; k++) {

            Eigen::Vector3d Pi_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Quaterniond Qi_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

            Eigen::Vector3d Pj_ck(parameters[1][0], parameters[1][1], parameters[1][2]);
            Eigen::Quaterniond Qj_ck(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

            Eigen::Quaterniond line_qua_ck( parameters[2][4],parameters[2][1],parameters[2][2],parameters[2][3]);
            double phi_ck = parameters[2][0];

            int a = k / 3, b = k % 3;
            Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

            if (a == 0)
                Pi_ck += delta;
            else if (a == 1)
                Qi_ck = Qi_ck * deltaQ(delta);
            else if (a == 2)
                Pj_ck += delta;
            else if (a == 3)
                Qj_ck = Qj_ck * deltaQ(delta);
            else if (a == 4) {           // line orth的前三个元素
                line_qua_ck = line_qua_ck * deltaQ(delta);
            }
            else if (a == 5) {           // line orth的最后一个元素
                double w1_ck_ = cos(phi_ck);
                double w2_ck_ = sin(phi_ck);
                Eigen::Matrix2d W_ck;
                W_ck << w1_ck_, -w2_ck_, w2_ck_, w1_ck_;
                Eigen::Matrix2d delta_W;
                delta_W << cos(eps), -sin(eps),sin(eps), cos(eps);
                W_ck = W_ck * delta_W;
                phi_ck = asin( W_ck(1,0) );
            }

            Eigen::Matrix3d line_R_ck = line_qua_ck.toRotationMatrix();

            double w1_ck = cos(phi_ck);
            double w2_ck = sin(phi_ck);

            Vector3d u1_ck = line_R_ck.col(0);
            Vector3d u2_ck = line_R_ck.col(1);

            Vector3d n_ck = w1_ck * u1_ck;
            Vector3d v_ck = w2_ck * u2_ck;

            Vector6d line_cami_ck ;
            line_cami_ck.head(3) = n_ck;
            line_cami_ck.tail(3) = v_ck;

            Eigen::Matrix3d Rwci_ck = Qi_ck.toRotationMatrix();
            Eigen::Vector3d twci_ck(Pi_ck);
            Vector6d line_w_ck = plk_to_pose(line_cami_ck, Rwci_ck, twci_ck);

            Eigen::Matrix3d Rwcj_ck = Qj_ck.toRotationMatrix();
            Eigen::Vector3d twcj_ck(Pj_ck);
            Vector6d line_camj_ck = plk_from_pose(line_w_ck, Rwcj_ck, twcj_ck);

            // 直线的投影矩阵K为单位阵
            Eigen::Vector3d nc_ck = line_camj_ck.head(3);
            double l_norm_ck = nc_ck(0) * nc_ck(0) + nc_ck(1) * nc_ck(1);
            double l_sqrtnorm_ck = sqrt(l_norm_ck);

            double e1_ck = obs_i(0) * nc_ck(0) + obs_i(1) * nc_ck(1) + nc_ck(2);
            double e2_ck = obs_i(2) * nc_ck(0) + obs_i(3) * nc_ck(1) + nc_ck(2);

            Eigen::Vector2d tmp_residual;
            tmp_residual(0) = e1_ck / l_sqrtnorm_ck;
            tmp_residual(1) = e2_ck / l_sqrtnorm_ck;
            tmp_residual = sqrt_info * tmp_residual;

            num_jacobian.col(k) = (tmp_residual - residual) / eps;

        }
        std::cout << "num_jacobian:\n"
                  << num_jacobian.block(0,0,2,6) << "\n"
                  << num_jacobian.block(0,6,2,6) << "\n"
                  << num_jacobian.block(0,12,2,4) << "\n"
                  << std::endl;



        std::cout << "----------------------------" << std::endl;
    }
/*
    // check jacobian
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    if(jacobians[3])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 22> num_jacobian;// 3 * 6 + 4
    for (int k = 0; k < 22; k++)
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * deltaQ(delta);
        else if (a == 6) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 7) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_ci = orth_to_plk(line_orth);
        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

        Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
        Eigen::Vector3d twbi(Pi);
        Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

        Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
        Eigen::Vector3d twbj(Pj);
        Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

        Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_cj.head(3);

        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}





//////////////////////////////////////////////////
Eigen::Matrix2d lineProjectionFactor_incamera::sqrt_info;
lineProjectionFactor_incamera::lineProjectionFactor_incamera(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_incamera::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

    Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
    Eigen::Vector3d twbi(Pi);
    Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

    Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
    Eigen::Vector3d twbj(Pj);
    Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

    Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_cj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

/*
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Matrix6d invTwbj;
            invTwbj << Rwbj.transpose(), -Rwbj.transpose()*skew_symmetric(twbj),
                 en::Matrix3d::Zero(),  Rwbj.transpose();
*/

            Matrix3d Rwcj = Rwbj * Rbc;
            Vector3d twcj = Rwbj * tbc + twbj;
            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_bi.head(3);
            Vector3d dbi = line_bi.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwbi * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwbi * skew_symmetric( nbi) - skew_symmetric(twbi) * Rwbi * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwbi * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwbj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwbj.transpose() * (nw + skew_symmetric(dw) * twbj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwbj.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Vector3d nci = line_ci.head(3);
            Vector3d dci = line_ci.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = -Rbc.transpose() * Rbjbi * skew_symmetric( Rbc * dci) + Rbc.transpose() * skew_symmetric(Rbjbi * Rbc * dci);   // Lc_t
            Matrix3d tmp = skew_symmetric(tcjci) * Rcjci;
            jaco_Lc_ex.block(0,3,3,3) = -Rcjci * skew_symmetric(nci) + skew_symmetric(Rcjci * nci)
                                        -tmp * skew_symmetric(dci) + skew_symmetric(tmp * dci);    // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = -Rcjci * skew_symmetric(dci) + skew_symmetric(Rcjci * dci);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[3]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lc_orth;
            jaco_Lc_orth.setZero();
            jaco_Lc_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lc_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lc_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lc_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_orth;
        }

    }
/*
    // check jacobian
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    if(jacobians[3])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 22> num_jacobian;// 3 * 6 + 4
    for (int k = 0; k < 22; k++)
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * deltaQ(delta);
        else if (a == 6) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 7) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_ci = orth_to_plk(line_orth);
        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

        Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
        Eigen::Vector3d twbi(Pi);
        Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

        Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
        Eigen::Vector3d twbj(Pj);
        Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

        Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_cj.head(3);

        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}


Eigen::Matrix2d lineProjectionFactor_instartframe::sqrt_info;
lineProjectionFactor_instartframe::lineProjectionFactor_instartframe(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_instartframe::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    Eigen::Vector4d line_orth( parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_ci.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[0]);


            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lci_orth;
            jaco_Lci_orth.setZero();
            jaco_Lci_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lci_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lci_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lci_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc  * jaco_Lci_orth;
        }

    }

    return true;
}
