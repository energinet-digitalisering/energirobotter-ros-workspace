# Kinematics Manager

A ROS 2 package for managing kinematics of humanoid robots, including URDF parsing and inverse kinematics (IK) for arms.

There's also a tool for visualising the reachability space for analysis purposes. 

## Nodes

All nodes using TRAC IK needds to have passed `"robot_description"` as a parameter. The easiest way is to do it via a launch file, see example in `launch/reachability_map.launch.py`.

### trac_ik_node 

This noe computes IK solutions for left- and right arm end-effectors using TRAC-IK.
It Subscribes to target pose topics for each arm, and publishes joint states to the `/joint_states` topic for use by controllers or visualization tools.

#### Topics

- **Subscribed:**  
  - `/left/target_pose` (`geometry_msgs/PoseStamped`): Target pose for the left hand  
  - `/right/target_pose` (`geometry_msgs/PoseStamped`): Target pose for the right hand

- **Published:**  
  - `/joint_states` (`sensor_msgs/JointState`): Joint positions for all managed ik chains

#### Parameters

- `base_link` (`string`, default: `"link_torso"`): The base link of the robot for kinematics calculations.
- `publish_rate` (`double`, default: `30.0`): Frequency (Hz) at which joint states are published.


### reachability_map_generator

This node generates a reachability map for a robot arm by sampling a 3D workspace and checking which points are reachable using inverse kinematics (TRAC-IK). The result is saved as a PLY point cloud file for visualization and analysis.

#### Parameters

- `base_link` (`string`, default: `"link_torso"`): The base link for kinematics calculations.
- `tip_link` (`string`, default: `"link_left_hand"`): The tip link (end-effector) to analyze.
- `step_size` (`double`, default: `0.02`): Resolution of the workspace grid in meters.

#### Output

- **File:**  
  - `pointcloud.ply`: ASCII PLY file listing all reachable points in the workspace.

#### Notes

- Progress is shown in the log as a percentage of points processed.
- The workspace volume and orientation are hardcoded but can be modified in the source.
- Each thread uses its own TRAC-IK manager instance for thread safety.
- If interrupted (Ctrl+C), the node will save the partial results and overwrite the `pointcloud.ply` file with the current progress.

