// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/* The program asks for move amount in 3 dimensions.
 * It creates a Cartesian trajectory to that relative position.
 * If a Cartesian trajectory is possible, then executes it.
 *
 * --Gokhan Solak
*/

#include <cartesian_move.h>


double arq_ur5::compute_relative_cartesian_move(moveit::planning_interface::MoveGroupInterface& move_group,
		moveit_msgs::RobotTrajectory& trajectory, geometry_msgs::Point& move_vec){

	std::string end_link = move_group.getEndEffectorLink();
	// get the current position
	geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

	// create a waypoint
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::Pose end_pose = start_pose;

	end_pose.position.x += move_vec.x;
	end_pose.position.y += move_vec.y;
  end_pose.position.z += move_vec.z;
  waypoints.push_back(end_pose);

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
	// FIXME: It's unreliable! IK can have jumps.
  const double jump_threshold = 6.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Cartesian path (%.2f%% acheived)", fraction * 100.0);

	return fraction;
}
