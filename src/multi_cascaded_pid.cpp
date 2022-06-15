#include <auv_control/multi_cascaded_pid.h>
#include <set>

namespace auv_control
{

void MultiCascadedPID::initControllers(const std::string &ns, std::chrono::milliseconds sampling_time)
{
  if(!initFromParams())
  {
    initFromModel(ns);
    // declare corresponding params
    for(const auto &pid: pids)
    {
      for(const auto &[gain, value]: pid.gainsParams())
        node->declare_parameter(gain, value);
      node->declare_parameter(pid.axis + ".use_position", pid.use_position);
    }
  }

  if(pids.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No pids are available, stopping node");
    rclcpp::shutdown();
  }

  for(auto &pid: pids)
    pid.setSamplingTime(sampling_time);

  printControllers();

  gains_callback = node->add_on_set_parameters_callback([&](const auto &parameters)
  {return tuneFromParams(parameters);});
}

rcl_interfaces::msg::SetParametersResult MultiCascadedPID::tuneFromParams(const std::vector<rclcpp::Parameter> &parameters)
{
  static rcl_interfaces::msg::SetParametersResult result;
  for(const auto &param: parameters)
  {
    for(auto &pid: pids)
    {
      if(pid.hasGain(param.get_name()))
      {
        result.reason += pid.tuneFromParam(param);
        break;
      }
    }
  }

  result.successful = result.reason.empty();
  return result;
}

std::optional<CascadedPID *> MultiCascadedPID::whoControls(const std::string &axis)
{
  auto pid{std::find_if(pids.begin(), pids.end(), [axis](const auto &pid)
    {return pid.axis == axis;})};
  if(pid != pids.end())
    return pid.base();
  else
    return {};
}

bool MultiCascadedPID::initFromParams()
{
  // check if we indeed have some axes as params
  const auto names{node->list_parameters({}, 2).names};
  auto gains{CascadedPID::defaultGains()};

  std::set<std::string> axes;
  for(const auto param: names)
  {
    auto dot = param.find('.');
    if(dot != param.npos)
    {
      const auto gain = param.substr(dot+1);
      if(gains.find(gain) != gains.end())
        axes.insert(param.substr(0, dot));
    }
  }

  if(axes.empty())
    return false;

  for(const auto &axis: axes)
  {
    for(auto &[gain, value]: gains)
      node->get_parameter_or(axis + "." + gain, value, 0.);
    pids.emplace_back(axis, gains);
  }
  return true;
}

}