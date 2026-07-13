# reBotArm Motor Control Model Implementation Plan

**Goal:** Make MuJoCo arm control follow the project's POS_VEL architecture and gripper control follow MIT using existing project configuration.

1. Add failing pure-controller tests for YAML/URDF loading, POS_VEL equations, saturation, anti-windup, and gripper MIT.
2. Implement a focused `motor_control.py` module with immutable parameters and stateful controllers.
3. Add failing generator tests requiring direct torque actuators with URDF force limits; regenerate `robot.xml`.
4. Integrate desired-position state and 100 Hz torque updates into `RebotArmMujoco` while preserving public APIs.
5. Update dynamics/ROS/viewer tests to validate tracking and mode semantics rather than position-actuator internals.
6. Run the full Windows suite, synchronize to Ubuntu VM, build, run EGL/headless/ROS 2 checks, merge, and push.

No real hardware is enabled or contacted.
