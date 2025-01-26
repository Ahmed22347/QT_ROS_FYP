/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <std_msgs/String.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>


//#include "open_manipulator_msgs/OpenManipulatorState.h"
//#include "open_manipulator_msgs/SetJointPosition.h"
//#include "open_manipulator_msgs/SetKinematicsPose.h"
//#include "open_manipulator_msgs/SetDrawingTrajectory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Float32.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <chrono>
#include <thread>


#include <vector>
#include <utility>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

//open_manipulator_msgs::KinematicsPose kinematics_pose_;

/*void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

 

  kinematics_pose_.pose = msg->pose;
}
*/

std::pair<float, float> convertPoint(const std::pair<float, float>& point) {
    // Example coefficients for linear transformation (These are simplistic and might need adjustment)
    //float a = -3.5/100, b = 22.5/100, c = 3.5/100, d = -12.25/100;
   // float a = -3.5/100, b = 28/100, c = 3.5/100, d = -12/100;
        //float a = (-28/700), b = 28/100, c = 24/700, d = -12/100;
    float a = -0.04;
    float b = 0.28;
    float c = -0.04;
    float d = 0.14;
        
    float world_x = a * point.first + b;
    float world_y = c * point.second + d;
    return std::make_pair(world_x, world_y);
}


void createCollisionObjectsFromURDF(const std::vector<std::pair<float, float>>& points,
                                    const std::string& frame_id,
                                    const std::string& mesh_resource,
                                    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> object_colors;
    for (const auto& point : points) {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "object_" + std::to_string(point.first) + "_" + std::to_string(point.second);
        ROS_INFO_STREAM("Attaching object: " << collision_object.id<< " to the robot.");

        // Load mesh from a URDF file
        shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        const std::pair<float, float> points = convertPoint(point);
        // Define the pose of the object
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = points.first;  // Use converted world coordinates as needed
        pose.position.y = points.second;
        pose.position.z = 0.0; // Height from the ground, adjust as necessary

        // Attach the mesh to the collision object
        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        moveit_msgs::ObjectColor color;
        color.id = collision_object.id;
        color.color.r = 1.0;  // Red
        color.color.g = 1.0;  // Green
        color.color.b = 1.0;  // Blue
        color.color.a = 1.0;  // Alpha (opacity)
        object_colors.push_back(color);

        // Add the collision object to the vector
        collision_objects.push_back(collision_object);
    }

    // Add collision objects to the planning scene
    planning_scene_interface.addCollisionObjects(collision_objects, object_colors);


}

void publishData(ros::Publisher& pub, float value) {
    std_msgs::Float32 msg;
    msg.data = value;
    pub.publish(msg);
    ROS_INFO("Published: %f", msg.data);
}

void pickAndPlace(const std::pair<float, float> initial_point,
                  const std::pair<float, float> final_point,
                  const std::string& object_id,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  ros::Publisher pub,
                  const std::string end_effector_link) {


    // Connect to Move Group
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 

    const std::pair<float, float> initial_vector = convertPoint(initial_point);
    tf2::Quaternion q_rot;
    geometry_msgs::Pose target_pose1;
    geometry_msgs::Pose target_pose2;
    // target_pose1.orientation.w = 1.000000;
    target_pose1.position.x = initial_vector.first;
    target_pose1.position.y = initial_vector.second;
    target_pose1.position.z = 0.02;

    double angle_z = atan2(target_pose1.position.y, target_pose1.position.x);
    move_group.setGoalOrientationTolerance(0.02);
    q_rot.setRPY(0, (M_PI/2), 0);
    tf2::convert(q_rot, target_pose1.orientation);
    move_group.setPoseTarget(target_pose1);

    // Move to the initial pose to reach the object
    move_group.move();  // Blocking call to ensure the move completes

    // Attach the object
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.link_name = end_effector_link;
    attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
    ROS_INFO_STREAM("Attaching object: " << object_id << " to the robot.");

    // The attach operation is also a blocking call
    publishData(pub, 15.0);
    planning_scene_interface.applyAttachedCollisionObject(attached_object);

      std::vector<geometry_msgs::Pose> waypoints1;
      waypoints1.push_back(target_pose1);



      const std::pair<float, float> final_vector = convertPoint(final_point);
      target_pose2.position.x = final_vector.first;
      target_pose2.position.y = final_vector.second;
      target_pose2.position.z = 0.02;
      angle_z = atan2(target_pose2.position.y, target_pose2.position.x);
      move_group.setGoalOrientationTolerance(0.01);
      q_rot.setRPY(0,( M_PI/2), 0);
      tf2::convert(q_rot, target_pose2.orientation);
      move_group.setPoseTarget(target_pose2);
      waypoints1.push_back(target_pose2);  // up and left

      // We want the Cartesian path to be interpolated at a resolution of 1 cm

      moveit_msgs::RobotTrajectory trajectory1;
      const double jump_threshold1 = 0.00;
      const double eef_step1 = 0.01;
      // move_group.setGoalOrientationTolerance(0.02);
      double fraction1 = move_group.computeCartesianPath(waypoints1, eef_step1, jump_threshold1, trajectory1);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction1 * 100.0);
        if (fraction1 == 1.0) {  // Check if the full path was planned successfully
        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
        my_plan1.trajectory_ = trajectory1;
        move_group.execute(my_plan1);
        } else {
          ROS_WARN("The Cartesian path was not completed entirely.");
        }



    // Optionally, detach and remove the object after placing it
    moveit_msgs::AttachedCollisionObject detach_object;
    publishData(pub, 15.0);
    detach_object.object.id = object_id;
    detach_object.link_name = end_effector_link;
    detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(detach_object);



  target_pose1.position.x = 0.082;
  target_pose1.position.y = -0.000;
  target_pose1.position.z = 0.114;

  // double angle_z = atan2(target_pose1.position.y, target_pose1.position.x);
  //  move_group_interface.setGoalOrientationTolerance(1.0);
   q_rot.setRPY(M_PI, (M_PI/2), M_PI);
  tf2::convert(q_rot, target_pose1.orientation);
  move_group.setPoseTarget(target_pose1);
      move_group.move();

    ROS_INFO("Operation completed successfully.");
}

void pickAndRotate(const std::pair<float, float> initial_point,
                  const std::string& object_id,
                  moveit::planning_interface::MoveGroupInterface& move_group,
                  moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                  const std::string end_effector_link,
                  ros::Publisher pub,
                  float rotation) {


    // Connect to Move Group
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 

    const std::pair<float, float> initial_vector = convertPoint(initial_point);
    tf2::Quaternion q_rot;
    geometry_msgs::Pose target_pose1;
    geometry_msgs::Pose target_pose2;
    // target_pose1.orientation.w = 1.000000;
    target_pose1.position.x = initial_vector.first;
    target_pose1.position.y = initial_vector.second;
    target_pose1.position.z = 0.02;

    //double angle_z = atan2(target_pose1.position.y, target_pose1.position.x);
    move_group.setGoalOrientationTolerance(0.02);
    q_rot.setRPY(0, (M_PI/2), 0);
    tf2::convert(q_rot, target_pose1.orientation);
    move_group.setPoseTarget(target_pose1);

    // Move to the initial pose to reach the object
    move_group.move();  // Blocking call to ensure the move completes

    // Attach the object
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.link_name = end_effector_link;
    attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
    ROS_INFO_STREAM("Attaching object: " << object_id << " to the robot.");

    // The attach operation is also a blocking call
    publishData(pub, 15.0);
    planning_scene_interface.applyAttachedCollisionObject(attached_object);
    tf2::Quaternion q_rot1;
    q_rot1.setRPY(0, (M_PI/2), rotation);
    tf2::convert(q_rot1, target_pose1.orientation);
    move_group.setPoseTarget(target_pose1);
    move_group.move();



    // Optionally, detach and remove the object after placing it
    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = object_id;
    detach_object.link_name = end_effector_link;
    detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(detach_object);
    publishData(pub, 15.0);



  target_pose1.position.x = 0.082;
  target_pose1.position.y = -0.000;
  target_pose1.position.z = 0.114;

  // double angle_z = atan2(target_pose1.position.y, target_pose1.position.x);
  //  move_group_interface.setGoalOrientationTolerance(1.0);
   q_rot.setRPY(M_PI, (M_PI/2), M_PI);
  tf2::convert(q_rot, target_pose1.orientation);
  move_group.setPoseTarget(target_pose1);
  move_group.move();

    ROS_INFO("Operation completed successfully.");
}




int main(int argc, char** argv)
{
  std::this_thread::sleep_for(std::chrono::seconds(10));
  ros::init(argc, argv, "new_trajectory");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  static const std::string PLANNING_GROUP = "arm";
  // static const std::string PLANNING_GROUP2 = "gripper";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  // moveit::planning_interface::MoveGroupInterface move_group_interface2(PLANNING_GROUP2);
    
    //  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // const moveit::core::JointModelGroup* joint_model_group2 =
  //     move_group_interface2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);

  // Visualization
  // ^^^^^^^^^^^^^
 // ros::Subscriber open_manipulator_kinematics_pose_sub_;
  //open_manipulator_kinematics_pose_sub_ = node_handle.subscribe("/gripper/kinematics_pose", 10, kinematicsPoseCallback);
      // Create a publisher object
  ros::Publisher pub = node_handle.advertise<std_msgs::Float32>("example_topic", 10);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  // .. _move_group_interface-planning-to-pose-goal:
  //

  tf2::Quaternion q_rot;
  geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.000000;
  target_pose1.position.x = 0.082;
  target_pose1.position.y = -0.000;
  target_pose1.position.z = 0.114;

  // double angle_z = atan2(target_pose1.position.y, target_pose1.position.x);
  //  move_group_interface.setGoalOrientationTolerance(1.0);
   q_rot.setRPY(M_PI, (M_PI/2), M_PI);
  tf2::convert(q_rot, target_pose1.orientation);
  move_group_interface.setPoseTarget(target_pose1);

  //move_group_interface.setPositionTarget(0.2,0,0.2,"end_effector_link");
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;  	
  //move_group_interface.setPlanningTime(20.0);
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

///////Adding object///////////

    move_group_interface.move();
    std::vector<std::pair<float, float>> points = {{4, 0}, {7, 7}, {3, 5}, {1, 2},{2,0},{5,3}};
    std::string mesh_resource = "package://open_manipulator_controllers/include/meshes/base_link.STL";  // Example mesh resource path

    createCollisionObjectsFromURDF(points, "world", mesh_resource, planning_scene_interface);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.publishText(text_pose, "Adding pickanplace ", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    pickAndPlace({4,0},{4,5}, "object_4.000000_0000000", move_group_interface,planning_scene_interface, pub, "end_effector_link");
    pickAndRotate({4,5}, "object_4.000000_1.000000", move_group_interface,planning_scene_interface, "end_effector_link", pub, (M_PI/2));


  //  pickAndPlace({7,7},{7,4}, "object_7.000000_7.000000", move_group_interface,planning_scene_interface, pub, "end_effector_link");
//    pickAndRotate({7,4}, "object_7.000000_7.000000", move_group_interface,planning_scene_interface, "end_effector_link", pub, (M_PI/2));
    
    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
   pickAndPlace({5,3},{2,3}, "object_5.000000_3.000000", move_group_interface,planning_scene_interface, pub, "end_effector_link");
    pickAndRotate({2,3}, "object_5.000000_3.000000", move_group_interface,planning_scene_interface, "end_effector_link", pub, (M_PI/2));


  ros::shutdown();
  return 0;
}
