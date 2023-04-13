#ifndef AUV_CONTROL_POS_VEL_PID_H
#define AUV_CONTROL_POS_VEL_PID_H

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


class PID
{
  friend class CascadedPID;
public:

  using Gains = std::map<std::string, double>;

  inline static Gains defaultGains()
  {
    return {{"Kp", 1}, {"Kv",.5}, {"Ki", .1}, {"Kd", 0}, {"v_sat",0}, {"u_sat",0}};
  }

  explicit PID(const std::string &axis, short component, Gains gains) :
    axis{axis}, component{component},
    Kp{gains["Kp"]}, Kv{gains["Kv"]}, Ki{gains["Ki"]}, Kd{gains["Kd"]},
    v_sat{gains["v_sat"]}, u_sat{gains["u_sat"]}
  { }

  static void setSamplingTime(double dt)
  {
    PID::dt = dt;
  }

  void tuneFromParam(const rclcpp::Parameter &param);

  // allow easy access
  bool use_position{true};
  std::string axis;
  short component;

  double update(double pos_error, double vel, double vel_sp);

  inline bool hasGain(const std::string &name) const
  {
    return name.rfind(axis+".", 0) == 0;
  }

  inline double integralTerm() const {return cmd_integral;}

private:
  double Kp{}, Kv{}, Ki{}, Kd{}, v_sat{}, u_sat{};

  double cmd_integral{0}, vel_prev{0};
  inline static double dt{0.1};

  double* gainFromName(const std::string &gain);
};

}

#endif // AUV_CONTROL_POS_VEL_PID_H
