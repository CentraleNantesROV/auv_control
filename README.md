# Basic control nodes for a AUV

This package contains two nodes to perform position and velocity control of an AUV.

## Dependencies

- [thruster_manager](https://github.com/CentraleNantesROV/thruster_manager) for the thrust allocation
- [simple_launch](https://github.com/oKermorgant/simple_launch) for the launch files
- [slider_publisher](https://github.com/oKermorgant/slider_publisher) for basic GUI to run tests

## Common properties

All controllers subscribe to the following:

- `cmd_pos` (`PoseStamped`): the pose setpoint expressed in the header frame
- `cmd_vel` (`TwistStamped`): the velocity setpoint expressed in the header frame
- `odom` (`Odometry`): an estimation of the current state (twist) of the vehicle
- `current_estim` (`Vector3`): an estimation of the ocean current in the world frame
- `/tf`: to get the transforms between all frames, also used to estimate the twist if `odom` is not available

The output may be:

- `cmd_thrust` (`JointState`): only the `effort` field is populated with the corresponding thrust for this thruster
- `<thruster>_cmd` (`Double`): the desired thrust for this thruster. This output is compatible with Gazebo thruster plugin.

## Cascaded PID

This controller takes 4 gains per axis: `Kp, Kd, Ki, Kv`. 

In addition, two saturation values may be given: `v_sat` (velocity saturation) and `u_sat` (overall wrench saturation). `u_sat` can be retrieved from the model of the robot if the max thrust is given.

The parameter `use_position`, given per-axis, tell if the position setpoint should be used for this component.

All gains and parameters are prefixed with the name of the axis: `x.Kp, x.Kv`, etc.

The overall control is as follows:

- without position control: velocity error `e = Kv.(v*-v)`
- with position control: velocity error `e = Kv.(v*-v) + sat(p*-p), -v_sat, v_sat)`
- output command `u = sat(Kp.e + Ki.integral(e) - Kd.derivative(e), -u_sat, u_sat)`
