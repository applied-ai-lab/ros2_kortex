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

With the default `soft_stop_position_hold=false` setting:

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
# FRANK TORQUE control

Branch `ros2-compliant` combines FRANK's `frank` driver at
`495ed5a4ebadda9dff997c92d828d60cf6373d50` with the effort implementation from
Exclaim `jazzy-rackthor` at `140f7965604730b7f753612e0521a8eafac1af2f`.
The selected backend is Kinova **TORQUE** mode; Exclaim's additional CURRENT loop is not included.

FRANK's soft-stop/watchdog and absent-gripper handling are retained. Effort entry seeds measured
torque; exit verifies POSITION restoration, with emergency-stop fallback on failure. Rejected
prepare calls do not block writes. Joint and Cartesian command modes exclude one another, and
FRANK effort mode requires all arm joints together. Reactivation reseeds measured pose/torque
after manual repositioning. An engaged soft stop rejects torque entry and fault reset; release
holds POSITION until an arm controller is explicitly reactivated.

Hardware testing of the earlier automatic-resume code showed the arm dropping during
the stop and snapping back on release. That code has been reverted. Post-revert
stop/release behavior still needs confirmation on the robot.

SDK headers and static library both use **2.8.0** on Linux amd64. Commands are total torque in
ROS joint signs and N·m; gravity belongs to `compliant_controllers`, not this driver. Each command
is limited by its URDF effort bound and `effort_command_max_step` (default 0.5 N·m per cycle).
Seven command signs are configured explicitly by the FRANK launch; the `+1` defaults are based
on Kinova's example and still require physical verification on FRANK.

Local `test_effort_interface` exercises actual mode preparation, command limiting, invalid-command
handling, absent-gripper reads and activation-state seeding without connecting to a robot.
`test_soft_stop` covers latch/watchdog and ROS topic behaviour. Actual SDK transitions, faults,
communication timing and physical compliance remain commissioning checks. See the accompanying
`compliant_controllers/docs/commissioning.md` and `docs/validation.md`.

## Powered soft stop for compliant bringup

FRANK compliant bringup sets `soft_stop_position_hold=true`. A button press captures
and holds the current pose using Kinova POSITION control. Release keeps the hold until
explicit controller reactivation. The driver exports `soft_stop/holding` so the effort
controller can abort actions and reject new motion commands while paused.

A failed hold handover is logged and latched for explicit recovery. The operator uses
the independent hardware hard stop. The hold request itself has no automatic emergency-stop
fallback. Existing hardware-fault and stop-heartbeat watchdog protections still apply.
The default setting is false, preserving the existing emergency-stop topic behavior.
This powered transition still requires verification on the robot under load.
