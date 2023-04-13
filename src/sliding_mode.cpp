#include <auv_control/controller_io.h>


namespace auv_control
{

using std::string;
using std::vector;

inline double sign(double v)
{
  return v > 0 ? 1 : -1;
}

enum class GainParam {VALID, NON_NUMERIC, NEGATIVE};
GainParam check(const rclcpp::Parameter &param)
{
  if(param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
     param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    return GainParam::NON_NUMERIC;
  if(param.as_double() < 0)
    return GainParam::NEGATIVE;
  return GainParam::VALID;
}

struct Gain : Vector6d
{
  std::string name;
  explicit Gain(const std::string &name): name(name) {}

  inline void setLin(double val)
  {
    head<3>().setConstant(val);
  }
  inline void setAng(double val)
  {
   tail<3>().setConstant(val);
  }

  void declareParam(rclcpp::Node* node, double lin=0, double ang=0)
  {
    setLin(node->declare_parameter(name + ".lin", lin));
    setAng(node->declare_parameter(name + ".ang", ang));
  }
  bool controls(const std::string &axis) const
  {
    return axis.rfind(name+".", 0) == 0;
  }
  std::string updateParam(const rclcpp::Parameter &param)
  {
    const auto req{check(param)};

    if(req == GainParam::VALID)
    {
      const auto axis{param.get_name().substr(name.size()+1)};
      if(axis == "lin")
        setLin(param.as_double());
      else
        setAng(param.as_double());
      return {};
    }
    return param.get_name() + (req == GainParam::NEGATIVE ? " is negative " : " is not numeric ");
  }
};

class SlidingMode : public ControllerIO
{
  // control gains
  Gain lambda{"lambda"};
  Gain eps{"eps"};
  Gain alpha{"alpha"};
  Gain Kbar{"kbar"};
  Gain mu{"mu"};
  Vector6d K;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr gains_callback;

public:
  SlidingMode() : ControllerIO("body_control_sm")
  {
    const auto ns = string{get_namespace()};

    lambda.declareParam(this, 10,10);
    eps.declareParam(this, 0.1, 0.1);
    alpha.declareParam(this, .01, .01);
    Kbar.declareParam(this,1,1);
    mu.declareParam(this,.1,.1);
    K.setConstant(declare_parameter("K", 0.5));

    gains_callback = add_on_set_parameters_callback([&](const auto &parameters)
    {return tuneFromParams(parameters);});
  }

  Vector6d computeWrench(const Vector6d &se3_error,
                         const Vector6d &vel,
                         const Vector6d &vel_setpoint) override
  {
    Vector6d S{vel_setpoint-vel};

    for(uint i = 0; i < 6; ++i)
    {
      // hybrid error
      S[i] += lambda[i]*se3_error[i];

      // update K
      if(K[i] > alpha[i])
        K[i] += dt * Kbar[i] * sign(std::abs(S[i])-mu[i]);
      else
        K[i] += dt * alpha[i];

      // saturate -> to wrench
      S[i] = K[i] * (std::abs(S[i]) > eps[i] ? sign(S[i]) : S[i]/eps[i]);
    }

    std::cout << "Current K: " << K.transpose() << std::endl;

    return S;
  }

private:

  rcl_interfaces::msg::SetParametersResult tuneFromParams(const std::vector<rclcpp::Parameter> &parameters)
  {
    static rcl_interfaces::msg::SetParametersResult result;

    for(const auto &param: parameters)
    {
      const auto &name{param.get_name()};
      if(name == "K")
      {
        const auto req{check(param)};
        if(req == GainParam::VALID)
          K.setConstant(param.as_double());
        else
          result.reason += std::string{"K"} + (req == GainParam::NEGATIVE ? " is negative " : " is not numeric ");
      }
      if(lambda.controls(name)) result.reason += lambda.updateParam(param);
      else if(eps.controls(name)) result.reason += eps.updateParam(param);
      else if(Kbar.controls(name)) result.reason += Kbar.updateParam(param);
      else if(alpha.controls(name)) result.reason += alpha.updateParam(param);
      else if(mu.controls(name)) result.reason += mu.updateParam(param);
    }

    result.successful = result.reason.empty();
    return result;
  }

};
}



// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_control::SlidingMode>());
  rclcpp::shutdown();
  return 0;
}
