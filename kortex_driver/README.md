# ROS 2 KINOVA KORTEX™ Driver
The ROS 2 KINOVA KORTEX™ driver implements the `ros2_control` hardware interface for a `SystemInterface`.

### Command interfaces
This driver exports commands interfaces for position, velocity, and effort interfaces for each joint defined in the URDF.
Additionally, twist interfaces are exported for the end effector for operational space control.
Several additional interfaces are exported, including `set_gripper_max_velocity`, `set_gripper_max_effort` for the gripper joint,
`reset_fault/command`, and `reset_fault/async_success` for fault management.

### State interfaces
This driver exports position and velocity state interfaces for joint defined in the URDF.

Additionally, one state interface `reset_fault/internal_fault` is used for determining the robot's fault state.

### Soft stop topic (optional)
Set the hardware parameter `soft_stop_topic` in the `<ros2_control>` block to have the driver follow a
soft stop published on a ROS topic, as the ROS 1 driver did with `/in/emergency_stop` and `/in/clear_faults`:

```xml
<hardware>
  <plugin>kortex_driver/KortexMultiInterfaceHardware</plugin>
  ...
  <param name="soft_stop_topic">/frank/soft_stop</param>
  <param name="soft_stop_watchdog">true</param>     <!-- optional, default true -->
  <param name="soft_stop_timeout_s">0.5</param>     <!-- optional, default 0.5 -->
</hardware>
```

| parameter | default | |
|---|---|---|
| `soft_stop_topic` | (empty = off) | `std_msgs/Bool` topic, true = stop |
| `soft_stop_watchdog` | `true` | silence on the topic counts as a stop |
| `soft_stop_timeout_s` | `0.5` | watchdog timeout [s]; the publisher must send its state more often (a heartbeat) |

| message on the topic (`std_msgs/Bool`) | the driver |
|---|---|
| `true` (stop) | applies the arm's emergency stop (`ApplyEmergencyStop`) once |
| `false` (release) | clears the faults (`ClearFaults`) and restores the servoing mode, once |

- Unset or empty: no subscription, the driver behaves as upstream.
- The subscription is reliable and transient local (depth 1), so the latest state of a latched soft stop
  arrives even if it was published before the driver started. Republishing the same state (e.g. at 10 Hz)
  causes no further calls; only a change between two control cycles does.
- After a release the arm **holds its measured position** and ignores the controllers' commands (joint,
  twist and gripper) until the next controller switch (`perform_command_mode_switch`, e.g. re-activating
  the arm's controller in `rqt_controller_manager`). A trajectory controller that kept running during the
  stop therefore cannot make the arm jump when the stop is lifted.
- While the soft stop is engaged, `reset_fault/command` is refused (the fault reset would also clear the
  emergency stop); release the soft stop instead.
- A failed Kortex call is logged and retried after 100 control cycles.
- The callback only stores the requested state; the Kortex calls happen in `write()` (as the fault reset).
  `ApplyEmergencyStop`/`ClearFaults` are blocking TCP calls, so the cycle in which they run is late.
- **Watchdog** (`soft_stop_watchdog`, default on): if no message arrives for longer than
  `soft_stop_timeout_s`, the driver applies the emergency stop, as if `true` had been received. The first
  deadline counts from activation, so the soft stop must be running (publishing) for the arm to move. The
  publisher must therefore republish its state periodically: a heartbeat (Frank's soft stop publishes every
  0.1 s, so 0.5 s tolerates four lost messages). When messages resume with `false`, the stop is released as
  above (faults cleared, arm holds until a controller switch). With the watchdog off, silence does nothing.
