# sas_robot_driver_denso
> The code uses the MIT licensed b-Cap source-code made available by DENSO WAVE INCORPORATED. For more information, refer to the source files in `src/bcap`.

Communicate with DENSO robots using the b-Cap protocol, by subclassing a `RobotDriver` from `sas_robot_driver`.

## Requirements

### Rosilo
- `sas_robot_driver`
- `sas_common`
- `sas_core`
- `rclcpp`

### DQ Robotics
See installation instructions at: https://dqrobotics.github.io/
- `libdqrobotics`

## ROS 2 node

The package installs the shared driver library and the
`sas_robot_driver_denso_node` executable. Starting the node connects to the
configured controller and, by default, initializes the robot, including enabling
its motors. Set the boolean parameter `read_only:=true` at startup to monitor
joint positions while the operator controls the robot with the teach pendant.
The default is `false`; this parameter cannot be changed while the node runs.

The following parameters are required and validated at startup in both modes
(`robot_speed` is unused in read-only mode):

| Parameter | Type | Meaning |
| --- | --- | --- |
| `robot_ip_address` | string | Nonempty controller address for UDP b-CAP |
| `robot_port` | integer | UDP port, 1–65535 |
| `robot_speed` | double | Positive finite value passed to b-CAP `ExtSpeed` |
| `thread_sampling_time_sec` | double | Positive finite control-loop period in seconds; replaces `thread_sampling_time_nsec` |
| `q_min` | double array | Six finite lower joint limits, in radians |
| `q_max` | double array | Six finite upper joint limits, in radians; each must exceed `q_min` |

The SAS topic prefix is the fully qualified node name: a node named `arm1`
publishes `/arm1/get/joint_states` and joint limits, and receives targets on
`/arm1/set/target_joint_positions`. Joint states and targets use radians.
With `read_only=false`, SIGINT/SIGTERM request the existing driver cleanup
before ROS shutdown; initialization and cleanup retain the original controller
behavior and delays.

With `read_only=true`, the node prints a warning and keeps the same SAS joint
topics, including the target-position subscription, but ignores all received
targets. Feedback and joint limits are still published in radians. The driver
does not take or give arm control, change motor state, set speed, enter or leave
slave mode, or send joint/Cartesian motion commands. Shutdown only releases the
connection resources. Monitoring also works at zero joint positions, without
the normal mode's startup wait for a joint-position norm of at least one radian.
Feedback has also been checked on a controller; motion under teach-pendant
control and other controller modes were not exercised.

The SAS-derived code retains [LGPL v3](LICENSE) and [GPL v3](LICENSE.GPL-3.0)
terms; the bundled b-CAP sources retain their [DENSO MIT notice](LICENSE.bcap).
