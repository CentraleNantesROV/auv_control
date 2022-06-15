#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <auv_control/eigen_typedefs.h>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml.h>

namespace auv_control
{

class ModelParser
{

  TiXmlElement* findNamedPlugin(const std::string &name) const
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
    return nullptr;
  }

  static inline double readTag(TiXmlElement *plugin, const std::string &key, double default_value = 0.)
  {
    const auto elem{plugin->FirstChildElement(key)};
    if(elem == nullptr) return default_value;
    return std::stod(elem->GetText());
  }

  std::string hydro_link;
  urdf::ModelInterfaceSharedPtr model;
  TiXmlDocument model_xml;
  TiXmlElement* root;


public:
  ModelParser(const std::string &hydro_link) : hydro_link{hydro_link}
  {
    const auto rsp_node(std::make_shared<rclcpp::Node>("hydrodynamics_rsp"));
    const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                               (rsp_node, "robot_state_publisher");
    rsp_param_srv->wait_for_service();

    if(!rsp_param_srv->has_parameter("robot_description"))
    {
      // cannot get the model anyway
      RCLCPP_ERROR(rsp_node->get_logger(), "cannot get model in namespace %s", rsp_node->get_namespace());
      return;
    }

    const auto xml{rsp_param_srv->get_parameter<std::string>("robot_description")};

    model_xml.Parse(xml.c_str());
    root = model_xml.RootElement();
    model = urdf::parseURDF(xml);

    if(model->links_.find(hydro_link) == model->links_.end())
      RCLCPP_ERROR(rsp_node->get_logger(), "cannot find link `%s` in robot_description %s",
                   hydro_link.c_str(),
                   rsp_node->get_namespace());
  }

  void parseStatics(Matrix6d &Mi, Eigen::Vector3d &cog) const
  {
    const auto inertial{model->getLink(hydro_link)->inertial};
    if(!inertial)
      return;

    cog(0) = inertial->origin.position.x;
    cog(1) = inertial->origin.position.y;
    cog(2) = inertial->origin.position.z;

    Mi.setZero();
    Mi(0,0) = Mi(1,1) = Mi(2,2) = inertial->mass;
    // inertia
    Mi(3,3) = inertial->ixx;
    Mi(4,4) = inertial->iyy;
    Mi(5,5) = inertial->izz;
    Mi(3,4) = Mi(4,3) = inertial->ixy;
    Mi(3,5) = Mi(5,3) = inertial->ixz;
    Mi(5,4) = Mi(4,5) = inertial->iyz;
  }

  void parseBuoyancy(double &buoyancy, Eigen::Vector3d &cob) const
  {

  }

  void parseHydrodynamics(Matrix6d &Ma, Vector6d &lin_dag, Vector6d &quad_drag) const
  {

  }




};






void urdf2Eigen(const urdf::Vector3 &src, Eigen::Vector3d &dst)
{
  dst = Eigen::Vector3d{src.x, src.y, src.z};
}

std::string IAUV::getModelXML(const std::string &ns, std::string caller)
{
  // retrieve full model through robot_state_publisher param service
  const auto rsp_node(std::make_shared<rclcpp::Node>(caller + "_rsp"));

  const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
                             (rsp_node, ns + "/robot_state_publisher");

  rsp_param_srv->wait_for_service();

  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    RCLCPP_ERROR(rsp_node->get_logger(), "cannot get model in namespace %s", ns.c_str());
    return "";
  }

  return rsp_param_srv->get_parameter<std::string>("robot_description");
}

urdf::ModelInterfaceSharedPtr IAUV::getModel(const std::string &ns, std::string caller)
{
  const auto model_xml{getModelXML(ns, caller)};

  if(model_xml.empty())
    return nullptr;

  return urdf::parseURDF(model_xml);
}

std::vector<std::string> IAUV::parseModel(const std::string &ns)
{
  // get thruster joints
  const auto xml{getModelXML(ns, "body")};
  const auto urdf{urdf::parseURDF(xml)};

  ThrusterJoints thrusters;
  for(const auto &[name, joint]: urdf->joints_)
  {
    if(name.find("thruster") != name.npos || name.find("fin") != name.npos)
      thrusters[name] = joint;
  }

  n_thr = thrusters.size();

  TiXmlDocument doc;
  doc.Parse(xml.c_str());
  auto root = doc.RootElement();

  parseThrusterMap(root, thrusters);
  parseStatics(urdf->root_link_->inertial);
  parseHydrodynamics(root, urdf->root_link_->name);

  std::vector<std::string> names;
  std::transform(thrusters.begin(), thrusters.end(), std::back_inserter(names),
                 [](const auto &joint){return joint.first;});
  return names;
}

void IAUV::parseThrusterMap(TiXmlElement* root, const ThrusterJoints &thrusters)
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

void IAUV::parseStatics(urdf::InertialConstSharedPtr inertial)
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

void IAUV::parseHydrodynamics(TiXmlElement* root, const std::string &base_link)
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

void IAUV::compensate(Vector6d &wrench, const Eigen::Quaterniond &q, const Vector6d &vel) const
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

void IAUV::solveWrench(const Vector6d &wrench, std::vector<double> &omega) const
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
