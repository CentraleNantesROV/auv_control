#include <auv_control/cascaded_pid.h>

using std::string;
using namespace auv_control;


CascadedPID::CascadedPID() : ControllerIO("cascaded_pid")
{
  PID::setSamplingTime(dt);

  // get max thrust per axis, useless to have a running PID on a non-controllable axis
  const std::vector<std::string> axes{"x", "y", "z", "roll", "pitch", "yaw"};
  const auto max_wrench{allocator.maxWrench()};

  for(int axis = 0; axis < 6; ++axis)
  {
    if(max_wrench(axis) < 1e-3) continue;

    // the AUV can move in this direction, check for any parameters
    auto gains{PID::defaultGains()};
    gains["u_sat"] = max_wrench(axis);
    gains["v_sat"] = std::numeric_limits<double>::infinity();

    for(auto &[gain, val]: gains)
    {
      if(gain.size() > 3) // u_sat, v_sat
        val = declare_parameter(axes[axis] + "." + gain, val);
      else                // other gains
        val = declareParameterBounded(axes[axis] + "." + gain, val, 0., 300., 0.1);
    }

    pids.emplace_back(axes[axis], axis, gains);

    // override position control mode
    pids.back().use_position = declare_parameter(axes[axis] + "." + "use_position", true);
  }

  gains_callback = add_on_set_parameters_callback([&](const auto &parameters)
  {return tuneFromParams(parameters);});
}

Vector6d CascadedPID::computeWrench(const Vector6d &se3_error,
                                    const Vector6d &vel,
                                    const Vector6d &vel_setpoint)
{ 
  static Vector6d wrench;
  wrench.setZero();
  // call PIDs
  for(auto &pid: pids)
  {
    const auto idx{pid.component};
    wrench[idx] = pid.update(se3_error[idx], vel[idx], vel_setpoint[idx]);
  }
  return wrench;
}


rcl_interfaces::msg::SetParametersResult CascadedPID::tuneFromParams(const std::vector<rclcpp::Parameter> &parameters)
{
  for(const auto &param: parameters)
  {
    for(auto &pid: pids)
    {
      if(pid.hasGain(param.get_name()))
        pid.tuneFromParam(param);
    }
  }
  return rcl_interfaces::msg::SetParametersResult().set__successful(true);
}


// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CascadedPID>());
  rclcpp::shutdown();
  return 0;
}
