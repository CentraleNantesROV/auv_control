#include <auv_control/pos_vel_pid.h>

using namespace auv_control;

double* PID::gainFromName(const std::string &gain)
{
  if(gain == "Kp") return &Kp;
  if(gain == "Kv") return &Kv;
  if(gain == "Ki") return &Ki;
  if(gain == "Kd") return &Kd;
  if(gain == "v_sat") return &v_sat;
  if(gain == "u_sat") return &u_sat;
  return nullptr;
}

void PID::tuneFromParam(const rclcpp::Parameter &param)
{
  const auto name{param.get_name().substr(axis.size()+1)};

  if(name == "use_position")
  {
    use_position = param.as_bool();
    return;
  }

  const auto gain{gainFromName(name)};
  if(!gain) return;

  *gain = param.as_double();
}

double PID::update(double pos_error, double vel, double vel_sp)
{
  // actual PID is here
  double e{Kv*(vel_sp - vel)};
  if(use_position)
    e += std::clamp(pos_error, -v_sat, v_sat);

  const auto iTerm{Ki*dt*e};
  if(std::abs(cmd_integral+iTerm) < u_sat)
    cmd_integral += iTerm;

  const auto cmd{cmd_integral + Kp*e - Kd/dt*(vel-vel_prev)};
  vel_prev = vel;
  return std::clamp(cmd, -u_sat, u_sat);
}
