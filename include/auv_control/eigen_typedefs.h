#ifndef AUV_CONTROL_EIGEN_TYPEDEFS_H
#define AUV_CONTROL_EIGEN_TYPEDEFS_H

#include <Eigen/Core>

namespace auv_control
{

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using MatrixX6d = Eigen::Matrix<double, Eigen::Dynamic, 6>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

}


#endif
