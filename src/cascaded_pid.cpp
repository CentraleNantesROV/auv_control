#include <auv_control/cascaded_pid.h>

using namespace auv_control;

double* CascadedPID::gainFromName(const std::string &gain)
{
  if(gain == "Kp") return &Kp;
  if(gain == "Kv") return &Kv;
  if(gain == "Ki") return &Ki;
  if(gain == "Kd") return &Kd;
  if(gain == "v_sat") return &v_sat;
  if(gain == "u_sat") return &u_sat;
  return nullptr;
}

std::string CascadedPID::tuneFromParam(const rclcpp::Parameter &param)
{
  const auto name{param.get_name().substr(axis.size()+1)};

  if(name == "use_position" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
  {
    use_position = param.as_bool();
    return {};
  }

  const auto gain{gainFromName(name)};
  if(!gain) return {};

  const auto is_numeric{param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
                      param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER};
  const auto is_valid{param.as_double() >= 0};

  if(is_numeric && is_valid)
  {
    *gain = param.as_double();
    return {};
  }
  return name + (is_numeric ? " is negative " : " is not numeric ");
}

double CascadedPID::update()
{
  // actual PID is here
  double e{Kv*(vel_sp - vel)};
  if(use_position)
    e += std::clamp(Kp*(position_sp-position), -v_sat, v_sat);

  if(std::abs(cmd_integral) < u_sat)
    cmd_integral += Ki*dt*e;

  const auto cmd{cmd_integral + e - Kd/dt*(vel-vel_prev)};

  vel_prev = vel;
  return std::clamp(cmd, -u_sat, u_sat);
}