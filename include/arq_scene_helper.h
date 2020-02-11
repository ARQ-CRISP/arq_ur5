// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)



#ifndef ARQ_SCENE_HELPER_H_
#define ARQ_SCENE_HELPER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

class ArqSceneHelper{
private:
	ros::NodeHandle* node_handle_;

	void create_box_(moveit_msgs::CollisionObject* collision_object, std::string id,
		 	float* dimensions, float* position);
	void apply_scene_diff_(moveit_msgs::PlanningScene* scene_change);

public:
	ArqSceneHelper(ros::NodeHandle* node_handle);
	~ArqSceneHelper();
	void create_ur5_scene();
};

#endif /* ARQ_SCENE_HELPER_H_*/
