#ifndef AUV_CONTROL_CASCADED_PID_H
#define AUV_CONTROL_CASCADED_PID_H

#include <chrono>
#include <string>
#include <map>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace auv_control
{

/*
 * This class defines a simple position / velocity cascaded PID
 * The position loop only consists in a single proportional gain Kp
 * Its output is saturated at the max velocity of the system
 * The velocity loop has the proportional gain Kv, derivative Kd and integral Ki
 */


class CascadedPID
{
  friend class MultiCascadedPID;
public:

  using Gains = std::map<std::string, double>;

  inline static Gains defaultGains()
  {
    return {{"Kp", 1}, {"Kv",.5}, {"Ki", .1}, {"Kd", 0}, {"v_sat",0}, {"u_sat",0}};
  }

  CascadedPID(const std::string &axis, double Kp, double Kv, double Ki, double Kd, double v_sat, double u_sat)
    : axis(axis), Kp(Kp), Kv(Kv), Ki(Ki), Kd(Kd), v_sat(v_sat), u_sat(u_sat)
  { }

  CascadedPID(const std::string &axis, Gains gains) :
    CascadedPID(axis, gains["Kp"],gains["Kv"],gains["Ki"],gains["Kd"],gains["v_sat"],gains["u_sat"])
  { }

  void setSamplingTime(std::chrono::milliseconds ms)
  {
    dt = ms.count()/1000.;
  }

  std::string tuneFromParam(const rclcpp::Parameter &param);

  // allow easy access
  bool use_position{true};
  double position{0}, position_sp{0};
  double vel{0}, vel_sp{0};
  std::string axis;

  double update();

  inline bool hasGain(const std::string &name) const
  {
    return name.rfind(axis+".", 0) == 0;
  }

  inline Gains gainsParams() const
  {
    std::vector<std::pair<std::string, double>> current{{"Kp", Kp},{"Kv", Kv},{"Ki", Ki},{"Kd", Kd},{"v_sat", v_sat},{"u_sat", u_sat}};
    for(auto &[name,val]: current)
      name = axis + "." + name;
    return Gains{current.begin(), current.end()};
  }

  inline double integralTerm() const {return cmd_integral;}

private:
  double Kp{}, Kv{}, Ki{}, Kd{}, v_sat{}, u_sat{};
  double dt;

  double cmd_integral{0}, vel_prev{0};

  double* gainFromName(const std::string &gain);
};

}

#endif // AUV_CONTROL_GENERIC_PID_H