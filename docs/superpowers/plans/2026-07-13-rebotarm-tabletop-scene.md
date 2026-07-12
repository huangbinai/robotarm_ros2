# reBotArm Tabletop Scene Implementation Plan

**Goal:** Present the generated URDF-based robot correctly mounted on a visible table with a useful home pose and camera view.

**Architecture:** Keep the robot root at URDF world origin. Move the table surface to `z=0`, place the floor below it, add table legs, and reset through a scene `home` keyframe. The generated `robot.xml` remains the only runtime robot model.

## Tasks

1. Add failing scene contracts for table/floor/base alignment, legs, cube placement, camera framing, and a valid home keyframe.
2. Add a failing simulator reset contract requiring the `home` keyframe.
3. Update `scene.xml` and `RebotArmMujoco.reset()` minimally to satisfy the contracts.
4. Verify the home pose is finite, within limits, free of unintended table/self contacts, and visually renderable.
5. Run the full Windows suite, synchronize to Ubuntu VM, build, run EGL/headless/ROS 2 checks, then merge and push `main`.

No hardware backend is started or contacted.
