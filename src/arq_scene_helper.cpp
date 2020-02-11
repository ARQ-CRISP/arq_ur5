// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "arq_scene_helper.h"

ArqSceneHelper::ArqSceneHelper(ros::NodeHandle* node_handle){

	this->node_handle_ = node_handle;

}

ArqSceneHelper::~ArqSceneHelper(){

}

void ArqSceneHelper::create_ur5_scene(){

	//Create the planning scene message
  ROS_INFO("Setting up the scene for UR5.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;


  // Create the scene obstacles
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^

  std::vector<moveit_msgs::CollisionObject> collision_objects;

	// table
  moveit_msgs::CollisionObject table_obj;
	table_obj.header.frame_id = "/world";

	float dims[] = {1.0, 3.0, 0.16};
	float pos[] = {0, 0, -0.07};
	create_box_(&table_obj, "table", dims, pos);
  planning_scene.world.collision_objects.push_back(table_obj);

	// wall
  moveit_msgs::CollisionObject wall_obj;
	wall_obj.header.frame_id = "/world";

	dims[0] = 0.3;
	dims[2] = 1.0;

	pos[0] = -0.45; // 30 cm excluding the wall width
	pos[2] = 0.45;
	create_box_(&wall_obj, "wall", dims, pos);
  planning_scene.world.collision_objects.push_back(wall_obj);

	// control box
  moveit_msgs::CollisionObject box_obj;
	box_obj.header.frame_id = "/world";
	dims[0] = 0.5;
	dims[1] = 0.46;
	dims[2] = 0.6;

	pos[0] = 0.15;
	pos[1] = 0.63; // 40 cm excluding the box length
	pos[2] = 0.25;
	create_box_(&box_obj, "control_box", dims, pos);

  planning_scene.world.collision_objects.push_back(box_obj);

  ros::Duration(2.0).sleep();

	// Apply the changes
	this->apply_scene_diff_(&planning_scene);
}

void ArqSceneHelper::create_box_(moveit_msgs::CollisionObject* collision_object, std::string name, float* dimensions, float* position ){

  // The id of the object is used to identify it.
  collision_object->id = name;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = dimensions[0];
	primitive.dimensions[1] = dimensions[1];
	primitive.dimensions[2] = dimensions[2];

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = position[0];
  box_pose.position.y = position[1];
  box_pose.position.z = position[2];

  collision_object->primitives.push_back(primitive);
  collision_object->primitive_poses.push_back(box_pose);
  collision_object->operation = collision_object->ADD;

}

void ArqSceneHelper::apply_scene_diff_(moveit_msgs::PlanningScene* scene_change){
	ROS_INFO("Applying scene changes");

	//Initiate service client
	ros::ServiceClient scene_diff_client =
      node_handle_->serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
	scene_diff_client.waitForExistence();

  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = *scene_change;
  scene_diff_client.call(srv);

}

int main(int argc, char **argv){

  ros::init(argc, argv, "arq_scene_helper");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(10.0);
  sleep_time.sleep();
  sleep_time.sleep();

	ArqSceneHelper scene_helper(&node_handle);
	scene_helper.create_ur5_scene();

  ros::shutdown();
  return 0;
}
