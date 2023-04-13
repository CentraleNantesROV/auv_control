#ifndef AUV_CONTROL_CASCADED_PID_H
#define AUV_CONTROL_CASCADED_PID_H

#include <auv_control/pos_vel_pid.h>
#include <auv_control/controller_io.h>
#include <rclcpp/node.hpp>

namespace auv_control
{

class CascadedPID : public ControllerIO
{
public:

  using Controllers = std::vector<PID>;

  CascadedPID();

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
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr gains_callback;

  Vector6d computeWrench(const Vector6d &se3_error,
                         const Vector6d &vel,
                         const Vector6d &vel_setpoint) override;

  rcl_interfaces::msg::SetParametersResult tuneFromParams(const std::vector<rclcpp::Parameter> &parameters);
};
}

#endif
