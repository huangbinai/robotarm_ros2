# reBotArm MuJoCo Motor Control Model Design

## Control modes

- Arm trajectory and hold commands use a 100 Hz cascaded POS_VEL controller.
- Arm MIT is not a normal trajectory mode; it is reserved for gravity-compensation entry and hold.
- Gripper motion, closing, contact force, and hold use MIT control.

## Sources of truth

- `src/rebotarm_bringup/config/arm.yaml`: arm POS_VEL and MIT firmware parameters.
- `src/rebotarm_bringup/config/gripper.yaml`: gripper MIT parameters.
- `src/rebotarm_moveit_config/config/rebotarm.urdf`: joint effort and position limits.

The simulation package must not maintain a second handwritten gain table.

## POS_VEL mapping

At 100 Hz, for each arm joint:

1. The position PI loop produces a desired velocity and clips it to `vlim`.
2. The velocity PI loop produces a normalized torque command and clips it to `[-1, 1]`.
3. The normalized command is multiplied by URDF `effort` to produce joint torque in N·m.
4. Both integrators use saturation-aware anti-windup and reset on simulation reset or mode change.

The normalized-torque interpretation is explicit because the repository does not contain the DaMiao firmware current-loop scale or motor torque constants. A future low-risk hardware calibration may replace the normalization scale without changing the controller interface.

## Gripper MIT mapping

Each finger uses `tau = kp * position_error + kd * velocity_error + tau_ff`, clipped by the URDF finger effort. Equality coupling continues to enforce equal and opposite finger motion. Grasp force feed-forward remains a separate command input.

## MJCF and API

Generated MJCF actuators are direct motor actuators with URDF force limits. Python retains position-target APIs while storing desired positions separately from instantaneous torque controls. `control_targets` continues to expose position targets, not `data.ctrl` torque values. ROS 2 interfaces and joint names remain unchanged.

## Safety and validation

No hardware is connected. Tests cover configuration provenance, control equations, saturation, anti-windup, 100 Hz update cadence, reset behavior, tracking, finite dynamics, and gripper contact/hold. Windows and Ubuntu VM must pass before merge.
