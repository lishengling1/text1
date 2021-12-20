#include "wifi_ekf.h"

#define EPS 	1e-6
#define R  		0.06
#define LANDA 	0.05168

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << 0, -q(2), q(1),
        q(2), 0, -q(0),
        -q(1), q(0), 0;
    return ans;
}

WifiEkf::WifiEkf():
    g(Eigen::Vector3d::Zero()),
    p(Eigen::Vector3d::Zero()),
    v(Eigen::Vector3d::Zero()),
    q(Eigen::Matrix3d::Identity()),
    pre_p(Eigen::Vector3d::Zero()), 
    P(Eigen::Matrix<double, 9, 9>::Zero())
{
}

void WifiEkf::init(double t, const Eigen::Vector3d &_g)
{
    cur_t = t;
    ROS_INFO_STREAM("init filter with g body: " << g.transpose());
    q = Eigen::Quaterniond::FromTwoVectors(_g, Eigen::Vector3d(0.0, 0.0, -1.0)).toRotationMatrix(); // R^w_b
    g = q * _g;
    ROS_INFO_STREAM("aligned g world: " << g.transpose());
    //p = Eigen::Vector3d(2, 0, 0);
    p = Eigen::Vector3d(10, 10, 10);
    pre_p = p;
    v.setZero();
}

void WifiEkf::predict(double t, const Eigen::Vector3d &linear_acceleration_body, const Eigen::Vector3d &angular_velocity_body)
{
    double dt = t - cur_t;
    cur_t = t;
    Eigen::Quaterniond dq(1,
                          angular_velocity_body(0) * dt / 2,
                          angular_velocity_body(1) * dt / 2,
                          angular_velocity_body(2) * dt / 2);
    dq.w() = sqrt(1 - dq.vec().transpose() * dq.vec() );
    q = (Eigen::Quaterniond(q) * dq).normalized().toRotationMatrix();


    Eigen::Vector3d linear_acceleration_world = q * linear_acceleration_body - g;
    v += linear_acceleration_world * dt;
    p += v * dt + 0.5 * linear_acceleration_world * dt * dt;
    /*
    state propagation 
    delta_p = v * dt
    delta_v = skewSymmetric(q * a) = -q * skewSymmetric(a) * dt * q
    delta_q = -skewSymmetric(w) * dt * q
    */
    // define F matrix,  corresponds to system matrix
    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Zero();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    F.block<3, 3>(3, 6) = -q * skewSymmetric(linear_acceleration_body);
    F.block<3, 3>(6, 6) = -skewSymmetric(angular_velocity_body);
    // define Q matrix, corresponds to process noise
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.block<3, 3>(0, 0) = acc_cov;
    Q.block<3, 3>(3, 3) = gyr_cov;
    // define G matrix, corresponds to observation matrix (within EKF)
    Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();
    G.block<3, 3>(3, 0) = -q;
    G.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    // propagate P matrix
    //P_k = F_{k-1} * P_{k-1} * F^T_{k-1} + Q_k
    P = (Eigen::Matrix<double, 9, 9>::Identity() + dt * F) * P * (Eigen::Matrix<double, 9, 9>::Identity() + dt * F).transpose() + (dt * G) * Q * (dt * G).transpose();
}

void WifiEkf::update(Eigen::Vector3d z)
{
    ROS_INFO_STREAM("P1: " << P.diagonal().transpose());
    ROS_INFO_STREAM("Measurement: " << measurement(p, pre_p).transpose());
    Eigen::Vector3d y = z - measurement(p, pre_p);
    ROS_INFO_STREAM("Residual: " << y.transpose());
    Eigen::Matrix<double, 3, 9> Hp = jacobian();
    //ROS_INFO_STREAM("Jacobian: " << Hp);
    Eigen::Matrix3d S = Hp * P * Hp.transpose() + W_cov;
    Eigen::Matrix<double, 9, 3> K = P * Hp.transpose() * S.inverse();
    P = (Eigen::Matrix<double, 9, 9>::Identity() - K * Hp) * P;
    Eigen::Matrix<double, 9, 1> dx = K * y;
    p += dx.segment<3>(0);
    ROS_INFO_STREAM("Position correction: " << dx.segment<3>(0).transpose());
    v += dx.segment<3>(3);
    q = (Eigen::Quaterniond(q) * Eigen::Quaterniond(1.0, dx(6) / 2, dx(7) / 2, dx(8) / 2)).normalized().toRotationMatrix();
    ROS_INFO_STREAM("P2: " << P.diagonal().transpose());
    pre_p = p;
}


Eigen::Vector3d WifiEkf::measurement(const Eigen::Vector3d &_p, const Eigen::Vector3d &_pre_p)
{
    // displacement measurement
    Eigen::Vector3d disp = _p - _pre_p;
    return disp;
}

Eigen::Matrix<double, 3, 9> WifiEkf::jacobian()
{
    Eigen::Vector3d eps_v[3];
    eps_v[0] = Eigen::Vector3d(EPS, 0.0, 0.0);
    eps_v[1] = Eigen::Vector3d(0.0, EPS, 0.0);
    eps_v[2] = Eigen::Vector3d(0.0, 0.0, EPS);

    Eigen::Matrix<double, 3, 9> J = Eigen::Matrix<double, 3, 9>::Zero();
    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3d tmp_p = measurement(p + eps_v[i], pre_p) - measurement(p - eps_v[i], pre_p);
        J(i, i) = tmp_p(i);
    }
    return J / (2 * EPS);
}
