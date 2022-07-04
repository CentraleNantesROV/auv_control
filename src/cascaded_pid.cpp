#include <auv_control/cascaded_pid.h>

using std::string;
using namespace auv_control;




CascadedPID::CascadedPID() : ControllerIO("cascaded_pid")
{
  PID::setSamplingTime(cmd_period);

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
      val = declare_parameter(axes[axis] + "." + gain, val);

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
/*
namespace auv_control
{

BodyPID() : ControllerIO("cascaded_pid")

class BodyPID : public ControllerIO, public CascadedPID
{
public:
  BodyPID() : ControllerIO("cascaded_pid"), CascadedPID(this)
  {
    const auto ns = string{get_namespace()};

    //initControllers(ns, cmd_period);
      }

  Vector6d computeWrench(const Vector6d &se3_error,
                         const Vector6d &vel,
                         const Vector6d &vel_setpoint) override
  {
    static const std::array<std::string, 6> axis{"x","y","z","roll","pitch","yaw"};

    Vector6d wrench;
    wrench.setZero();
    // call PIDs
    for(uint i = 0; i < 6; ++i)
    {
      const auto pid_opt{whoControls(axis[i])};
      if(!pid_opt.has_value()) continue;
      auto & pid{*(pid_opt.value())};

      pid.vel = vel[i];
      pid.vel_sp = vel_setpoint[i];
      pid.position_sp = se3_error[i]; // current position is always 0 as we are in local frame
      wrench[i] = pid.update();
    }
    return wrench;
  }

private:

  void initFromModel([[maybe_unused]] const string &ns) override
  {
    const auto max_wrench{allocator.maxWrench()};

    const std::array<string,6> axes{{"x","y","z","roll","pitch","yaw"}};

    for(size_t i = 0; i < 6; ++i)
    {
      const auto &axis{axes[i]};

      if(max_wrench[i] > 1e-3)
      {
        // the AUV can move in this direction
        pids.emplace_back(axis, 1., 0.4, 0.1, 0, std::numeric_limits<double>::infinity(), max_wrench[i]);
      }
    }
  }


};
}
*/


// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CascadedPID>());
  rclcpp::shutdown();
  return 0;
}
