#include "optimi/line_parameterization.hpp"
#include "line_uti/line_geometry.hpp"


bool LineOrthParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{

    // theta --> U,  phi --> W
    Eigen::Map<const Eigen::Vector3d> theta(x);
    double phi = *(x + 3);
    //Vector3d theta = orth.head(3);
    //double phi = orth[3];
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

    // update
    Eigen::Map<const Eigen::Vector3d> _delta_theta(delta);
    double _delta_phi = *(delta + 3);
    Eigen::Matrix3d Rz;
    Rz << cos(_delta_theta(2)), -sin(_delta_theta(2)), 0,
            sin(_delta_theta(2)), cos(_delta_theta(2)), 0,
            0, 0, 1;

    Eigen::Matrix3d Ry;
    Ry << cos(_delta_theta(1)), 0., sin(_delta_theta(1)),
            0., 1., 0.,
            -sin(_delta_theta(1)), 0., cos(_delta_theta(1));

    Eigen::Matrix3d Rx;
    Rx << 1., 0., 0.,
            0., cos(_delta_theta(0)), -sin(_delta_theta(0)),
            0., sin(_delta_theta(0)), cos(_delta_theta(0));
    R = R * Rx * Ry * Rz;
//    R = R * Rz * Ry * Rx;

    Eigen::Matrix2d W;
    W << w1, -w2, w2, w1;
    Eigen::Matrix2d delta_W;
    delta_W << cos(_delta_phi), -sin(_delta_phi),sin(_delta_phi), cos(_delta_phi);
    W = W * delta_W;

    // U' -- > theta'. W' --> phi'
    Eigen::Map<Eigen::Vector3d> theta_pluse(x_plus_delta);                             // double 指针 转为eigen数组
    double* phi_plus(x_plus_delta + 3);

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);
    Vector3d u3 = R.col(2);
    theta_pluse[0] = atan2( u2(2),u3(2) );
    theta_pluse[1] = asin( -u1(2) );
    theta_pluse[2] = atan2( u1(1),u1(0) );

    *phi_plus = asin( W(1,0) );


    /*
    Eigen::Map<const Eigen::Vector4d> theta(x);
    Eigen::Map<const Eigen::Vector4d> _delta_theta(delta);
    Eigen::Map<Eigen::Vector4d> theta_pluse(x_plus_delta);

    theta_pluse = theta + _delta_theta;
*/
    /*
    Eigen::Map<const Eigen::Vector3d> theta(x);
    double phi = *(x + 3);
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

    Sophus::SO3 U = Sophus::SO3::exp(theta);
    Sophus::SO3 U1(R);

    std::cout << U.matrix() << "\n\n" <<U1.matrix()<<"\n\n"<<R<<"\n\n";

    std::cout << theta <<"\n\n" << U1.log() << "\n\n"<<  Sophus::SO3::exp(U1.log()).matrix() << "\n\n";
     */
    return true;
}
bool LineOrthParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();

    return true;
}

//template <typename Derived>
//static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &u_theta)
//{
//    typedef typename Derived::Scalar Scalar_t;
//
//    Eigen::Quaternion<Scalar_t> dq;
//    Scalar_t theta = u_theta.norm();
//    Eigen::Matrix<Scalar_t, 3, 1> u = u_theta.normalized();
//    Scalar_t half_theta = theta / static_cast<Scalar_t>(2.0);
//    dq.w() = static_cast<Scalar_t>(std::cos(half_theta));
//    dq.x() = static_cast<Scalar_t>(std::sin(half_theta)) * u(0);
//    dq.y() = static_cast<Scalar_t>(std::sin(half_theta)) * u(1);
//    dq.z() = static_cast<Scalar_t>(std::sin(half_theta)) * u(2);
//    return dq;
//}

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

bool LineOrthQuaParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // theta --> U,  phi --> W
    double phi = *(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x+1);

    double _delta_phi = *(delta);
    Eigen::Map<const Eigen::Vector3d> dp(delta+1);

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta+1));

    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta+1);

    q = (_q * dq).normalized();


    double w1 = cos(phi);
    double w2 = sin(phi);
    Eigen::Matrix2d W;
    W << w1, -w2, w2, w1;
    Eigen::Matrix2d delta_W;
    delta_W << cos(_delta_phi), -sin(_delta_phi),sin(_delta_phi), cos(_delta_phi);
    W = W * delta_W;

    double* phi_plus(x_plus_delta);
    *phi_plus = asin( W(1,0) );

    return true;
}
bool LineOrthQuaParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 5, 4, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    j.bottomRows(1).setZero();

    return true;
}


// TODO: 这里有问题啊，。，。，。，。，。，
// error state [tehta, d]
bool LineYangParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector4d> _q(x);

//    _q = _q.normalized();

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));
    double _delta_d = *(delta+3);
    dq = dq.coeffs() * _delta_d;
    Eigen::Vector4d delta_q( dq.x(), dq.y(), dq.z(), dq.w() );

    Eigen::Map<Eigen::Vector4d> q(x_plus_delta);
    q = _q + delta_q;

    return true;
}
bool LineYangParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> j(jacobian);
    j.setZero();

    return true;
}
