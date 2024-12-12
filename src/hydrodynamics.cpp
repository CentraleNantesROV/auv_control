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

Hydrodynamics::Hydrodynamics(const std::string &hydro_link, rclcpp::Node* node)
{
  const auto model{ModelParser(hydro_link)};
  model.parseStatics(Mi, cog);  
  model.parseBuoyancy(buoyancy, cob);
  model.parseHydrodynamics(Ma, lin_drag, quad_drag);

  if(node)
  {
    RCLCPP_INFO(node->get_logger(), "Got weight %.02f @ (%.02f %.02f %.02f)",
                Mi(0,0), cog(0), cog(1), cog(2));
    RCLCPP_INFO(node->get_logger(), "Got buoyancy %.02f @ (%.02f %.02f %.02f)",
                buoyancy, cob(0), cob(1), cob(2));
    RCLCPP_INFO(node->get_logger(), "Got linear drag %.02f %.02f %.02f %.02f %.02f %.02f",
                lin_drag(0,0), lin_drag(1,0), lin_drag(2,0),
                lin_drag(3,0), lin_drag(4,0), lin_drag(5,0));
    RCLCPP_INFO(node->get_logger(), "Got quadratic drag %.02f %.02f %.02f %.02f %.02f %.02f",
                quad_drag(0,0), quad_drag(1,0), quad_drag(2,0),
                quad_drag(3,0), quad_drag(4,0), quad_drag(5,0));
  }
}

void Hydrodynamics::compensate(Vector6d &wrench, const Eigen::Quaterniond &bRw,
                               const Vector6d &twist, const Vector3d &current) const
{
  // statics from gravity and buoyancy in local frame
  const auto grav{bRw * Eigen::Vector3d{0,0,-9.81*Mi(0,0)}};
  const auto buoy{bRw * Eigen::Vector3d{0,0,buoyancy}};
  wrench.head<3>() -= grav + buoy;
  wrench.tail<3>() -= cog.cross(grav) + cob.cross(buoy);

  // write current in body frame
  const auto rel_vel{twist.head<3>() - bRw*current};
  // drag
  for(uint i = 0; i < 3; ++i)
  {
    wrench(i) -= lin_drag(i)*(rel_vel(i)) + quad_drag(i)*std::abs(rel_vel(i))*rel_vel(i);
    wrench(i+3) -= lin_drag(i+3)*twist(i+3) + quad_drag(i+3)*std::abs(twist(i+3))*twist(i+3);
  }

  // Ma / Mi maybe later

}

}
