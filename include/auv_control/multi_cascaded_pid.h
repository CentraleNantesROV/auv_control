#ifndef AUV_CONTROL_MULTI_CASCADED_PID_H
#define AUV_CONTROL_MULTI_CASCADED_PID_H

#include <auv_control/cascaded_pid.h>
#include <rclcpp/node.hpp>

namespace auv_control
{

class MultiCascadedPID
{
public:

  using Controllers = std::vector<CascadedPID>;

  MultiCascadedPID(rclcpp::Node* node): node(node) {}

  void initControllers(const std::string &ns, std::chrono::milliseconds sampling_time);

  template <class Duration = std::chrono::milliseconds>
  inline Duration samplingTime()
  {
    if(pids.empty())
      return Duration{0};
    return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(int(pids[0].dt*1000)));
  }

  // to debug initialized PIDs
  inline void printControllers()
  {
    for(const auto &pid: pids)
    {
      printf("- %s: Kp = %f, Kv = %f, Ki = %f, Kd = %f, v_sat = %f, u_sat = %f\n",
             pid.axis.c_str(), pid.Kp, pid.Kv, pid.Ki, pid.Kd, pid.v_sat, pid.u_sat);
    }
  }

protected:

  Controllers pids;
  rclcpp::Node* node;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr gains_callback;

  rcl_interfaces::msg::SetParametersResult tuneFromParams(const std::vector<rclcpp::Parameter> &parameters);
  std::optional<CascadedPID*> whoControls(const std::string &axis);

  bool initFromParams();
  virtual void initFromModel(const std::string&) = 0;

};



}

#endif