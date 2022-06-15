#include <auv_control/hydrodynamics.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "model_parser.h"

using std::cout;
using std::endl;
using std::string;

namespace auv_control
{

Hydrodynamics::Hydrodynamics(const std::string &hydro_link)
{

  const auto model{ModelParser(hydro_link)};

  model.parseStatics(Mi, cog);
  model.parseBuoyancy(buoyancy, cob);
  model.parseHydrodynamics(Ma, lin_drag, quad_drag);
}

void Hydrodynamics::compensate(Vector6d &wrench, const Eigen::Quaterniond &q, const Vector6d &vel) const
{
  // statics from gravity and buoyancy in local frame
  const auto grav{q.conjugate() * Eigen::Vector3d{0,0,-9.81*Mi(0,0)}};
  const auto buoy{q.conjugate() * Eigen::Vector3d{0,0,buoyancy}};
  wrench.head<3>() -= grav + buoy;
  wrench.tail<3>() -= cog.cross(grav) + cob.cross(buoy);

  // drag
  for(uint i = 0; i < 6; ++i)
    wrench(i) -= lin_drag(i)*vel(i) + quad_drag(i)*std::abs(vel(i))*vel(i);

  // Ma / Mi maybe later

}

}
