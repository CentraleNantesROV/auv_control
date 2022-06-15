#include <auv_control/controller_io.h>
#include <auv_control/multi_cascaded_pid.h>


namespace auv_control
{

using std::string;
using std::vector;

class BodyPID : public ControllerIO, public MultiCascadedPID
{
public:
  BodyPID() : ControllerIO("body_control_pid"), MultiCascadedPID(this)
  {
    const auto ns = string{get_namespace()};

    initControllers(ns, cmd_period);
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



// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_control::BodyPID>());
  rclcpp::shutdown();
  return 0;
}