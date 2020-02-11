// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/* The program asks for move amount in 3 dimensions.
 * It creates a Cartesian trajectory to that relative position.
 * If a Cartesian trajectory is possible, then executes it.
 *
 * --Gokhan Solak
*/

#include <cartesian_move.h>

using namespace arq_ur5;

std::string manipulator_name_;
std::string end_link_name_;

bool getParams();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_move");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // attempt to get ros parameters
  if(!getParams()) return -1;

  // Setup move group interface
  moveit::planning_interface::MoveGroupInterface move_group(manipulator_name_);
	// set velocity scaler
	move_group.setMaxVelocityScalingFactor(0.1);
	move_group.setMaxAccelerationScalingFactor(0.1);


	geometry_msgs::Point move_vec;
	double fraction;
	while(ros::ok()){
		// synchronize with ros loop
		ros::spinOnce();
		//
		std::cout << "Cartesian movement amount (cm)";
		std::cout << "\nx: ";
		std::cin >> move_vec.x;

		std::cout << "\ny: ";
		std::cin >> move_vec.y;

		std::cout << "\nz: ";
		std::cin >> move_vec.z;

		// convert to meters
		move_vec.x /= 100;
		move_vec.y /= 100;
		move_vec.z /= 100;

		//compute the desired move
		moveit_msgs::RobotTrajectory cartesian_traj;
		fraction = compute_relative_cartesian_move(move_group, cartesian_traj, move_vec);

		// execute the computed trajectory if successful
		// success : more than 90% of the path is achieved
		if(fraction > 0.9){

      std::string ans;
      std::cout << "Execute? (y/n) ";
      std::cin >> ans;

      // execute if user confirms
      if(ans=="y"){
  			moveit::planning_interface::MoveGroupInterface::Plan plan;
  			plan.trajectory_ = cartesian_traj;
  			move_group.execute(plan);
      }
		}else {
			ROS_WARN("Trajectory is not executed because it was not possible.");
		}
	}

  ros::shutdown();
  return 0;
}

bool getParams(){

  std::string param_name;

	// manipulator name
  if(!ros::param::get("~manipulator", manipulator_name_)){
    ROS_WARN("Cartesian move: Can't get manipulator param.");
		return false;
  }

  ROS_DEBUG("Cartesian move: manipulator is %s.", manipulator_name_.c_str());

  // end link name
  if(!ros::param::get("~end_link", end_link_name_)){
    ROS_WARN("Cartesian move: Can't get end_link param.");
		return false;
  }

  ROS_DEBUG("Cartesian move: end_link is %s.", end_link_name_.c_str());
  return true;

}
