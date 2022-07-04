#include <auv_control/controller_io.h>
#include <chrono>

using namespace auv_control;
using namespace std::chrono_literals;

void twist2Eigen(const geometry_msgs::msg::Twist &twist, Vector6d &vel)
{
  vel[0] = twist.linear.x;
  vel[1] = twist.linear.y;
  vel[2] = twist.linear.z;
  vel[3] = twist.angular.x;
  vel[4] = twist.angular.y;
  vel[5] = twist.angular.z;
}

void wrench2Eigen(const geometry_msgs::msg::Wrench &wrench, Vector6d &vec)
{
  vec[0] = wrench.force.x;
  vec[1] = wrench.force.y;
  vec[2] = wrench.force.z;
  vec[3] = wrench.torque.x;
  vec[4] = wrench.torque.y;
  vec[5] = wrench.torque.z;
}

Vector6d ControllerIO::Pose::toSE3() const
{
  Vector6d se3_error;
  se3_error.head<3>() = t;

  const auto sin_theta_over_2{q.vec().norm()};
  if(sin_theta_over_2 < 1e-6)
    se3_error.tail<3>().setZero();
  else
  {
    auto angle{2*atan2(sin_theta_over_2, q.w())};
    if(angle > M_PI)  angle -= 2*M_PI;
    else if(angle < -M_PI)  angle += 2*M_PI;
    se3_error.tail<3>() = q.vec() * angle/sin_theta_over_2;
  }
  return se3_error;
}

ControllerIO::ControllerIO(std::string name, rclcpp::NodeOptions options)
  : Node(name,
         options//.automatically_declare_parameters_from_overrides(true)
         .allow_undeclared_parameters(true)),
    tf_buffer(get_clock()), tf_listener(tf_buffer)
{
  // control_frame defaults to ns/base_link
  control_frame = get_namespace();

  if(const auto slash{control_frame.find_last_of('/')}; slash == control_frame.npos)
    control_frame = "base_link";
  else
    control_frame.substr(slash+1) + "/base_link";
  control_frame = declare_parameter<std::string>("control_frame", control_frame);

  // control output
  cmd.name = allocator.parseRobotDescription(this, control_frame);
  dofs = cmd.name.size();
  cmd.effort.resize(dofs, 0);
  if(declare_parameter("publish_joint_state", true))
    cmd_js_pub = create_publisher<JointState>("cmd_thrust", 5);

  if(get_parameter("use_sim_time").as_bool())
  {
    for(const auto &name: cmd.name)
    {
      const auto topic{"cmd_" + name};
      cmd_gz_pub.push_back(create_publisher<Float64>(topic,5));
    }
  }

  // control input
  pose_sub = create_subscription<PoseStamped>("cmd_pose", 10, [&](PoseStamped::SharedPtr msg)
  {          pose_setpoint.from(msg->pose.position, msg->pose.orientation);
             pose_setpoint.frame = msg->header.frame_id;
             pose_setpoint.time = get_clock()->now().seconds();});

  vel_sub = create_subscription<TwistStamped>("cmd_vel", 10, [&](TwistStamped::SharedPtr msg)
  {
            twist2Eigen(msg->twist, vel_setpoint);
            vel_setpoint.frame = msg->header.frame_id;
            vel_setpoint.time = get_clock()->now().seconds();});

  wrench_sub = create_subscription<Wrench>("cmd_wrench", 1, [&](Wrench::SharedPtr msg)
  {
               wrench2Eigen(*msg, wrench_setpoint);
               wrench_setpoint.time = get_clock()->now().seconds();});

  odom_sub = create_subscription<Odometry>("odom", 10, [&](Odometry::SharedPtr msg)
  {twist2Eigen(msg->twist.twist, vel);
             Pose::tf2Rotation(msg->pose.pose.orientation, orientation);});

   control_srv = create_service<ControlMode>
                ("control_mode",
                 [&](const ControlMode::Request::SharedPtr request,
                 [[maybe_unused]] ControlMode::Response::SharedPtr response)
  {control_mode = request->mode;});

  const auto cmd_period_ms{declare_parameter("control_period_ms", 100)};
  cmd_period = std::chrono::milliseconds(cmd_period_ms);
  cmd_timer = create_wall_timer(cmd_period, [&]() {publish(computeThrusts());});


}

Eigen::VectorXd ControllerIO::computeThrusts()
{
  // correct setpoints according to timestamps
  const auto now_s{get_clock()->now().seconds()};
  const auto pose_timeout{now_s - pose_setpoint.time > pose_setpoint_timeout};
  const auto vel_timeout{now_s - vel_setpoint.time  > vel_setpoint_timeout};
  const auto wrench_timeout{now_s - wrench_setpoint.time > pose_setpoint_timeout};

  if(pose_timeout && vel_timeout && wrench_timeout)
  {
    // no more setpoints
    return Eigen::VectorXd::Zero(dofs);
  }

  if(!wrench_timeout)
  {
    // manual wrench control overrides navigation
    if(hydro)
      hydro->compensate(wrench_setpoint, orientation, vel);
    return allocator.solveWrench(wrench_setpoint);
  }

  // pose / vel control
  Vector6d se3_error;
  if(pose_timeout || control_mode == ControlMode::Request::VELOCITY)
    se3_error.setZero();
  else if(pose_setpoint.frame == control_frame)
    se3_error = pose_setpoint.toSE3();
  else // to vehicle frame
    se3_error = (relPose(pose_setpoint.frame) * pose_setpoint).toSE3();

  Vector6d vel_setpoint_local;
  if(vel_timeout)
    vel_setpoint_local.setZero();
  else if(vel_setpoint.frame == control_frame || vel_setpoint.isApproxToConstant(0, 1e-3))
    vel_setpoint_local = vel_setpoint;
  else // to vehicle frame
    relPose(vel_setpoint.frame).rotate2(vel_setpoint, vel_setpoint_local);

  // hybrid control mode
  if(control_mode == ControlMode::Request::DEPTH)
  {
    // position mode only for Z, roll, pitch
    se3_error[0] = se3_error[1] = se3_error[5] = 0.;
  }

  auto wrench{computeWrench(se3_error, vel, vel_setpoint_local)};
  if(hydro)
    hydro->compensate(wrench, orientation, vel);
  return allocator.solveWrench(wrench);
}

void ControllerIO::publish(const Eigen::VectorXd &thrusts)
{
  std::copy(thrusts.data(), thrusts.data()+dofs, cmd.effort.begin());
  if(cmd_js_pub)
  {
    cmd.header.stamp = get_clock()->now();
    cmd_js_pub->publish(cmd);
  }

  if(!cmd_gz_pub.empty())
  {
    static Float64 cmd_gz;
    for(size_t i = 0; i < dofs; ++i)
    {
      cmd_gz.data = thrusts[i];
      cmd_gz_pub[i]->publish(cmd_gz);
    }
  }
}

ControllerIO::Pose ControllerIO::relPose(const std::string &frame)
{
  Pose rel_pose;
  if(tf_buffer.canTransform(control_frame, frame, tf2::TimePointZero, 10ms))
  {
    const auto tf{tf_buffer.lookupTransform(control_frame, frame, tf2::TimePointZero, 10ms).transform};
    rel_pose.from(tf.translation, tf.rotation);
  }
  return rel_pose;
}
