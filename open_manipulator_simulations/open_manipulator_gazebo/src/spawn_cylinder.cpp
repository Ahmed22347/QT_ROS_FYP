#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <streambuf>

int main(int argc, char** argv) {
  ros::init(argc, argv, "spawn_cylinder");
  ros::NodeHandle nh;
  
  ros::service::waitForService("gazebo/spawn_sdf_model", ros::Duration(30));
  ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");


  gazebo_msgs::SpawnModel spawn_model;

  // Define the model pose
  geometry_msgs::Pose model_pose;
  model_pose.position.x = 0.4;
  model_pose.position.y = 0.4;
  model_pose.position.z = 0.05; // Height/2 + ground plane thickness

  // Load cylinder model from an SDF file
  std::ifstream model_file("spawn_cylinder.sdf");
  std::string model_xml((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

  // Set the properties of the service message
  spawn_model.request.model_name = "cylinder1";
  spawn_model.request.model_xml = model_xml;
  spawn_model.request.robot_namespace = "test_cylinder";
  spawn_model.request.initial_pose = model_pose;
  spawn_model.request.reference_frame = "world";
  // Wait for the 'spawn_sdf_model' service to become available

  if (spawn_client.call(spawn_model)) {
    ROS_INFO("Cylinder Spawned Successfully!");
  } else {
    ROS_ERROR("Failed to spawn cylinder. Error message: %s", spawn_model.response.status_message.c_str());
  }

  return 0;
}
