#ifndef AUV_CONTROL_HYDRODYNAMICS_H
#define AUV_CONTROL_HYDRODYNAMICS_H

#include <auv_control/eigen_typedefs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/node.hpp>

namespace auv_control
{

class Hydrodynamics
{
  double buoyancy;
  Vector6d max_wrench,quad_drag, lin_drag;
  Matrix6d Ma, Mi;
  Eigen::Vector3d cog, cob;

public:

  Hydrodynamics(const std::string &hydro_link, rclcpp::Node* node = nullptr);
  void compensate(Vector6d &wrench, const Eigen::Quaterniond &bRw,
                  const Vector6d &twist, const Vector3d& current) const;

};
}

#endif // AUV_CONTROL_HYDRODYNAMICS_H
