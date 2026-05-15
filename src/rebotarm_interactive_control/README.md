# rebotarm_interactive_control

ROS2 interactive-control package for the reBotArm 3D drag-control workflow.

## Recommended entrypoints

### Formal system

Use the split-node formal system for normal testing and integration:

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py
```

This starts:

- `MarkerServerNode`
- `PreviewNode`
- `ExecutionNode`
- `reBotArmController`
- `robot_state_publisher`
- `rviz2`

### Debug / smoke test

Use the dedicated debug launch for isolated marker interaction checks:

```bash
ros2 launch rebotarm_interactive_control interactive_debug.launch.py
```

## Transitional / legacy entrypoints

These remain only for compatibility during refactoring and should not be used as the default workflow:

- `ros2 launch rebotarm_bringup interactive_basic.launch.py`
- `InteractiveTargetNode`

The preferred architecture is the split-node chain:

```text
MarkerServerNode -> PreviewNode -> ExecutionNode
```
