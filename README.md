# Adaptive Docking implementation
## This repo is an implementation of the Adaptive Docking (AD), including:
- adaptive_docking_local_planner: AD APIs can be used for the plugin with MBF
- docking_planner: the AD planner is to manage AD states, generate a docking path, and output control signal
- docking_control: the AD controller
- fuzzy_control, pure_pursuit_control: control algorithms
- utils: utilities
## Usage of APIs:
- initialize: initialize the AD, need to provide
  - tf2_ros::Buffer &tf: a server buffer used for internal tf transform
  - double sec: tf transform timeout
- setLocalFrame: set the local frame, should be a robot base frame, default: base_link_p
- setGlobalFrame: set the global frame, can be odom or map, depended on which frame is more reliable
- setPlan: set necessary inputs, including
  - std_msgs::Header &header: header of a goal pose (can be ignored)
  - geometry_msgs::PoseStamped &starting_pose: a pre-engagement pose (optional)
  - geometry_msgs::PoseStamped &approaching_pose: a goal pose for the first stage (required)
  - geometry_msgs::PoseStamped &docking_pose: the final docking pose (required)
- ExecuteControlLoop: start the AD, need input speed info
  - geometry_msgs::TwistStamped &velocity: velocity input (optional), with the speed input, the performance is better, without it, acceptable performance
- setGoalRange: set the range (in meter) inside it, the AD starts to check for goal reaching (optional), default 0.1
- IsApproachingReached: return true if the approaching is finished, false if not (optional because existed internal checking)
- IsGoalReached: return true if the AD is finished, false if not (required)
  - dx: tolerance of x (m)
  - dy: tolerance of y (m)
  - dyaw: tolerance of angle (m)
- getDockingResult: return the docking result code
