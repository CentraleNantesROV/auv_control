#include <auv_control/cascaded_pid.h>
#include <set>

namespace auv_control
{

void CascadedPID::initControllers(const std::string &ns, std::chrono::milliseconds sampling_time)
{
  if(!initFromParams())
  {
    initFromModel();
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



}
