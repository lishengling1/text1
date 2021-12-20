#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/console.h>

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q);

class WifiEkf
{
public:
    WifiEkf();
    void init(double t, const Eigen::Vector3d &_g);
    void predict(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void update(Eigen::Vector3d disp);
    Eigen::Vector3d measurement(const Eigen::Vector3d &_p, const Eigen::Vector3d &_pre_p);
    Eigen::Matrix<double, 3, 9> jacobian();

    Eigen::Vector3d g; //gravity

    double cur_t;
    Eigen::Vector3d p, v; //position, velocity
    Eigen::Matrix3d q; //rotation
    Eigen::Vector3d pre_p;  // the position at the previous wifi measurement 

    Eigen::Matrix<double, 9, 9> P; //cov of about state, without AP

    const Eigen::Matrix3d acc_cov = 0.1 * 0.01 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d gyr_cov = 0.01 * 0.001 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d W_cov = 1e-6 * Eigen::Matrix3d::Identity();
};
