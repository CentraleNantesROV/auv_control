#ifndef AUV_CONTROL_CONTROLLER_IO_H
#define AUV_CONTROL_CONTROLLER_IO_H

#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <thruster_manager/thruster_manager.h>

#include <Eigen/Geometry>

#include <auv_control/eigen_typedefs.h>
#include <auv_control/hydrodynamics.h>
#include <auv_control/srv/control.hpp>
#include <auv_control/butterworth.h>

namespace auv_control
{

class ControllerIO : public rclcpp::Node
{
  using Vector3 = geometry_msgs::msg::Vector3;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using JointState = sensor_msgs::msg::JointState;
  using Float64 = std_msgs::msg::Float64;
  using Wrench = geometry_msgs::msg::Wrench;
  using Odometry = nav_msgs::msg::Odometry;
  using ControlMode = auv_control::srv::Control;

public:
  ControllerIO(std::string name, rclcpp::NodeOptions options = rclcpp::NodeOptions{});

protected:

  inline double declareParameterBounded(const std::string &name, double default_value,
                          double min, double max, double step = 0.1)
  {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.name = name;
      descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                         .set__from_value(min)
                                         .set__to_value(max)
                                         .set__step(step)
                                        };
      return declare_parameter(name, default_value, descriptor);
  }
  inline int declareParameterBounded(const std::string &name, int default_value,
                          int min, int max, int step = 1)
  {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.name = name;
      descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()
                                         .set__from_value(min)
                                         .set__to_value(max)
                                         .set__step(step)
                                        };
      return declare_parameter(name, default_value, descriptor);
  }

  struct Pose
  {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;

    Pose() {}
    Pose(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) : t(t), q(q) {}

    inline static void tf2Rotation(const geometry_msgs::msg::Quaternion &q, Eigen::Quaterniond &qe)
    {
      qe.x() = q.x;
      qe.y() = q.y;
      qe.z() = q.z;
      qe.w() = q.w;
      qe.normalize();
    }
    inline Pose operator*(const Pose &other) const
    {
      return {t + q*other.t, q*other.q};
    }
    inline void rotate2(const Vector6d &src, Vector6d &dst) const
    {
      dst.head<3>() = q*src.head<3>();
      dst.tail<3>() = q*src.tail<3>();
    }
    template<class Translation>
    void from(const Translation &t, const geometry_msgs::msg::Quaternion &q)
    {
      this->t.x() = t.x;
      this->t.y() = t.y;
      this->t.z() = t.z;
      tf2Rotation(q, this->q);
    }
    Vector6d toSE3() const;
  };

  template<class Setpoint>
  struct StampedSetpoint : public Setpoint
  {
    std::string frame;
    double time{0};
  };

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  Pose relPose(const std::string &frame) const;

  // setpoints
  double align_thr{-1.};
  std::string control_frame;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub;
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub;
  rclcpp::Subscription<Wrench>::SharedPtr wrench_sub;

  StampedSetpoint<Pose> pose_setpoint;
  StampedSetpoint<Vector6d> vel_setpoint;
  StampedSetpoint<Vector6d> wrench_setpoint;

  // velocity
  Vector6d twist(const Eigen::Quaterniond &q);
  Butterworth_nD vel_filter;
  rclcpp::Subscription<Vector3>::SharedPtr current_sub;
  Vector3d current_estim;

  // odom estim
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  bool state_ok{false};
  std::optional<Vector6d> odom_twist;

  // command computation
  uint dofs{};
  double dt;
  rclcpp::Service<ControlMode>::SharedPtr control_srv;
  decltype (ControlMode::Request::mode) control_mode;
  rclcpp::TimerBase::SharedPtr cmd_timer;
  thruster_manager::ThrusterManager allocator;
  std::optional<Hydrodynamics> hydro;

  // command output
  // if we publish output as joint states  
  sensor_msgs::msg::JointState cmd;
  rclcpp::Publisher<JointState>::SharedPtr cmd_js_pub;

  // if we publish output as Float64 (Gazebo thruster plugin)
  std::vector<rclcpp::Publisher<Float64>::SharedPtr> cmd_gz_pub;

  // command output
  Eigen::VectorXd computeThrusts();
  void publish(const Eigen::VectorXd &thrusts);

  // actual-controller-dependent
  virtual Vector6d computeWrench(const Vector6d &se3_error,
                                 const Vector6d &vel,
                                 const Vector6d &vel_setpoint) = 0;

  // timeouts
  double pose_setpoint_timeout{1};
  double vel_setpoint_timeout{.1};
};
}

#endif // AUV_CONTROL_CONTROLLER_IO_H
