// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/* An interface to create Cartesian path
 * for the robot for given Cartesian distances.
 *
 * --Gokhan Solak
*/
#ifndef CARTESIAN_MOVE_H_
#define CARTESIAN_MOVE_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Point.h>

namespace arq_ur5{

double compute_relative_cartesian_move(moveit::planning_interface::MoveGroupInterface& move_group,
		moveit_msgs::RobotTrajectory& trajectory, geometry_msgs::Point& move_vec);

}

#endif /* CARTESIAN_MOVE_H_*/
