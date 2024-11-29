#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <auv_control/eigen_typedefs.h>
#include <urdf/model.h>
#include <tinyxml2.h>

namespace auv_control
{

class ModelParser
{

  tinyxml2::XMLElement* findNamedPlugin(const std::string &name) const
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

  static inline double readTag(tinyxml2::XMLElement *plugin, const std::string &key, double default_value = 0.)
  {
    if(!plugin) return default_value;

    if(const auto elem{plugin->FirstChildElement(key.c_str())}; elem != nullptr)
      return std::stod(elem->GetText());

    return default_value;
  }

  std::string hydro_link;
  urdf::Model model;
  tinyxml2::XMLDocument model_xml;
  tinyxml2::XMLElement* root;


public:
  ModelParser(const std::string &hydro_link) : hydro_link{hydro_link}
  {
    const auto rsp_node{std::make_shared<rclcpp::Node>("hydrodynamics_rsp")};
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
    model.initString(xml);

    if(model.links_.find(hydro_link) == model.links_.end())
      RCLCPP_ERROR(rsp_node->get_logger(), "cannot find link `%s` in robot_description %s",
                   hydro_link.c_str(),
                   rsp_node->get_namespace());
  }

  void parseStatics(Matrix6d &Mi, Eigen::Vector3d &cog) const
  {
    const auto inertial{model.getLink(hydro_link)->inertial};
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
    // for now only consider basic collision volume of the given link
    const auto collision{model.getRoot()->collision};
    if(!collision || collision->geometry->type == urdf::Geometry::MESH)
    {
      buoyancy = 0;
      return;
    }

    cob(0) = collision->origin.position.x;
    cob(1) = collision->origin.position.y;
    cob(2) = collision->origin.position.z;

    auto volume{0.};
    if(collision->geometry->type == urdf::Geometry::BOX)
    {
      const auto box{static_cast<urdf::Box*>(collision->geometry.get())};
      volume = box->dim.x * box->dim.y * box->dim.z;
    }
    else if(collision->geometry->type == urdf::Geometry::SPHERE)
    {
      const auto sphere{static_cast<urdf::Sphere*>(collision->geometry.get())};
      volume = 4*M_PI*pow(sphere->radius,3)/3.;
    }
    else if(collision->geometry->type == urdf::Geometry::CYLINDER)
    {
      const auto cylinder{static_cast<urdf::Cylinder*>(collision->geometry.get())};
      volume = M_PI*cylinder->length * pow(cylinder->radius,2);
    }

    // get density from any thruster
    const auto thruster{findNamedPlugin("systems::Thruster")};
    const auto density{readTag(thruster, "fluid_density", 1000)};

    buoyancy = density * volume * 9.81;
  }

  void parseHydrodynamics(Matrix6d &Ma, Vector6d &lin_drag, Vector6d &quad_drag) const
  {
    Ma.setZero();
    const auto hydro{findNamedPlugin("systems::Hydrodynamics")};
    if(!hydro)
    {
      lin_drag.setZero();
      quad_drag.setZero();
      return;
    }

    uint idx{0};
    for(const auto &[dir,comp]: std::vector<std::array<std::string,2>>{
    {"x", "U"},{"y", "V"},{"z", "W"},
    {"k", "P"},{"m", "Q"},{"n","R"}})
    {
      Ma(idx,idx) = readTag(hydro, dir+"Dot"+comp);
      lin_drag(idx) = readTag(hydro, dir+comp);
      quad_drag(idx) = readTag(hydro, dir+comp+"abs"+comp);
      idx++;
    }
  }
};

}
