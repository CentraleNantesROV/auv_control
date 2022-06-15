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








}

TiXmlElement* findNamedPlugin(TiXmlElement* root, const std::string &name)
{
  for(auto gz = root->FirstChildElement("gazebo");
      gz != nullptr;
      gz = gz->NextSiblingElement("gazebo"))
  {
    for(auto plugin = gz->FirstChildElement("plugin");
        plugin != nullptr;
        plugin = plugin->NextSiblingElement("plugin"))
    {
      const std::string plugin_name{plugin->Attribute("name")};
      if(plugin_name.find(name) != plugin_name.npos)
        return plugin;
    }
  }
}

void urdf2Eigen(const urdf::Vector3 &src, Eigen::Vector3d &dst)
{
  dst = Eigen::Vector3d{src.x, src.y, src.z};
}


urdf::ModelInterfaceSharedPtr Hydrodynamics::getModel(const std::string &ns, std::string caller)
{
  const auto model_xml{getModelXML(ns, caller)};

  if(model_xml.empty())
    return nullptr;

  return urdf::parseURDF(model_xml);
}


void Hydrodynamics::parseThrusterMap(TiXmlElement* root, const ThrusterJoints &thrusters)
{
  Matrix6Xd tam(6, n_thr);

  size_t thr{0};
  for(const auto &[name, joint]: thrusters)
    tam.col(thr++) = TAMcol(*joint);

  // build max thrust per-thruster
  max_thrusts.resize(n_thr);
  rotor_constants.resize(n_thr);

  for(auto plugin: findNamedPlugins(root, "libuuv_thruster_ros_plugin.so"))
  {
    if(auto joint = plugin->FirstChildElement("jointName");
       joint != nullptr && thrusters.find(joint->GetText()) != thrusters.end())
    {
      const auto idx{readFromTag<uint>(plugin, "thrusterID")};
      if(!readFromTag(plugin, "thrustMax", max_thrusts[idx]))
        max_thrusts[idx] = 0;
      if(!readFromTags(plugin, {"conversion", "rotorConstant"}, rotor_constants[idx]))
        rotor_constants[idx] = 0.003;
    }
  }

  // build inverse map
  tam_pinv.resize(n_thr, 6);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(tam, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Vector6d dummy_in;
  Eigen::VectorXd dummy_out(n_thr);
  unsigned int i,j;
  for(i=0;i<6;++i)
  {
    dummy_in.setZero();
    dummy_in(i) = 1;
    dummy_out = svd.solve(dummy_in);
    for(j = 0; j<n_thr;++j)
      tam_pinv(j,i) = dummy_out(j);
  }

  // find corresponding max wrench
  for(size_t dir = 0; dir < 6; ++dir)
  {
    max_wrench(dir) = 0.;
    for(size_t t = 0; t < n_thr; ++t)
      max_wrench(dir) += max_thrusts[t] * std::abs(tam(dir, t));
  }
}

void Hydrodynamics::parseStatics(urdf::InertialConstSharedPtr inertial)
{
  // cog mass
  urdf2Eigen(inertial->origin.position, cog);
  Mi(0,0) = Mi(1,1) = Mi(2,2) = inertial->mass;

  // inertia
  Mi(3,3) = inertial->ixx;
  Mi(4,4) = inertial->iyy;
  Mi(5,5) = inertial->izz;
  Mi(3,4) = Mi(4,3) = inertial->ixy;
  Mi(3,5) = Mi(5,3) = inertial->ixz;
  Mi(5,4) = Mi(4,5) = inertial->iyz;
}

void Hydrodynamics::parseHydrodynamics(TiXmlElement* root, const std::string &base_link)
{
  for(auto plugin: findNamedPlugins(root, "libuuv_underwater_object_ros_plugin.so"))
  {
    if(auto link{plugin->FirstChildElement("link")};
       link != nullptr && link->Attribute("name") == base_link)
    {
      // net buoyancy + CoB
      const auto density{readFromTag<double>(plugin, "fluid_density")};
      const auto volume{readFromTag<double>(link, "volume")};
      buoyancy = volume * density * 9.81;
      readMatrix(link, {"center_of_buoyancy"}, cob);

      readMatrix(link, {"hydrodynamic_model", "added_mass"}, Ma);
      readMatrix(link, {"hydrodynamic_model", "linear_damping"}, lin_drag);
      readMatrix(link, {"hydrodynamic_model", "quadratic_damping"}, quad_drag);
    }
  }
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

void Hydrodynamics::solveWrench(const Vector6d &wrench, std::vector<double> &omega) const
{
  // map to thrusts
  const auto thr{tam_pinv * wrench};

  //saturate
  double scale{1.};
  for(uint i = 0; i < n_thr; ++i)
    scale = std::max(scale, std::abs(thr(i))/max_thrusts[i]);
  if(scale > 1.)
  {
    for (uint i = 0; i < n_thr; ++i)
      omega[i] = thr(i)/scale;
  }
  else
  {
    for (uint i = 0; i < n_thr; ++i)
      omega[i] = thr(i);
  }

  // to angular velocity
  for(uint i = 0; i < n_thr; ++i)
  {
    if(omega[i] > 0)
      omega[i] = sqrt(omega[i])/rotor_constants[i];
    else
      omega[i] = -sqrt(-omega[i])/rotor_constants[i];
  }
}

}
