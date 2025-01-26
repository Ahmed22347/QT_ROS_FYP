#ifndef ROBOTMANIPULATOR_H
#define ROBOTMANIPULATOR_H

#include <ros/ros.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <utility>
#include <vector>
#include <QObject>
#include <QtConcurrent/QtConcurrent>
#include "twoplayerboard.h"
#include <std_msgs/Int32.h>  // Add this to your header or source file


using namespace std;

struct Vector3D {
    float x, y, z;
};

class RobotManipulator : public QObject {
    Q_OBJECT

public:
   explicit RobotManipulator(ros::NodeHandle& nh, std::string arm_number, std::string arm_number_ns, std::string robot_description, ros::Publisher pub_, bool simulation_mode = false, QObject* parent = nullptr);


    ~RobotManipulator();

    void pickAndPlace(Step step, const std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE> grid);
    void pickAndRotate(Step step, const std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE> grid);
    void checkBus(Step& step, const std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE>& grid);
    
// Correct declaration in robotmanipulator.h
void createCollisionObjectsFromURDF(const std::vector<Position>& positions,
                                    const std::string& frame_id,
                                    const std::string& car_mesh_resource,
                                    const std::string& bus_mesh_resource,
                                    const std::array<std::array<Piece, GRID_SIZE>, GRID_SIZE> grid);

    void publishData(int value);





signals:
    void operationCompleted();

private:
    void resetPose();
    std::pair<float, float> convertPoint(const Position& position);
    void executeTask(std::function<void()> task);
    std::string arm_number_ns; 
    std::string robot_description;
    void gotoPose(int number);	
    std::string arm_number;
    ros::NodeHandle& node_handle_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    ros::Publisher pub_;
    bool simulation_mode_;
    void restrictJointMovement(moveit::planning_interface::MoveGroupInterface& move_group); 
    
        const QString colorMatrix[8][8] = {
        {"B", "B", "LB", "BL", "BL", "LB", "LB", "LB"},
        {"B", "B", "GB", "BL", "BL", "GB", "LB", "LB"},
        {"LB", "LB", "GB", "Gr", "Gr", "GB", "LB", "LB"},
        {"LB", "LB", "Gr", "Gr", "Gr", "Gr", "LB", "LB"},
        {"LG", "LG", "Gr", "Gr", "Gr", "Gr", "LG", "LG"},
        {"LG", "LG", "GG", "Gr", "Gr", "GG", "LG", "LG"},
        {"LG", "LG", "GG", "BL", "BL", "GG", "G", "G"},
        {"LG", "LG", "LG", "BL", "BL", "LG", "G", "G"}
    };
    
    

            std::vector<std::vector<Vector3D>> matrix_1 = {
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },  // Row 1
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },  // Row 2
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },  // Row 3
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0/100, +1/100, -0.1/100}, {0, 0, 0}, {0, 0, 0} },  // Row 4
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 1.0/100, 0}, {0/100, 1.0/100, -0.1/100}, {0, 0, 0}, {0, 0, 0} },  // Row 5
        { {0, 0/100, 0}, {0, -0.15/100, 0.2}, {0, -0.1/100, 0}, {0, 0, 0}, {0, 0.5/100, 0}, {1/100, 1.0/100, 0.2/100}, {0.8/100, 1.0/100, 0}, {1.0, 1.2/100, 0.1/100} },  // Row 6
        { {0, +0.8/100, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1.5/100, 1.2/100, 0} },  // Row 7
        { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1.8/100, 0.8/100, 0} }   // Row 8
    };
    
    
std::vector<std::vector<Vector3D>> matrix_2 = {

    // Row 1 with updated z = 1.0 / 100.0
    { {(-1.5 / 100.0), (0.6 / 100.0), (1.1 / 100.0)}, {(-0.5 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(-0.5 / 100.0), (0.5 / 100.0), (1.0 / 100.0)}, {(-0.5 / 100.0), (0.5 / 100.0), (1.0 / 100.0)} },  // Row 1

    // Row 2 with updated z = 1.0 / 100.0
    { {(-1.5 / 100.0), (0.0 / 100.0), (1.55 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.2 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.2 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.2 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.2 / 100.0)}, {(0.5 / 100.0), (-0.7 / 100.0), (1.2 / 100.0)}, {(-0.8 / 100.0), (-0.2 / 100.0), (1.2 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (1.2 / 100.0)} },  // Row 2

{ {(-2.0 / 100.0), (0.5 / 100.0), (1.6/ 100.0)}, {(-1.8 / 100.0), (0.3 / 100.0), (1.4 / 100.0)}, {(-1.4 / 100.0), (0.0 / 100.0), (1.3/ 100.0)}, {(-1.8 / 100.0), (-0.2 / 100.0), (1.0 / 100.0)}, {(-1.5 / 100.0), (0 / 100.0), (1.0/ 100.0)}, {(-1.1 / 100.0), (0.0 / 100.0), (0.95 / 100.0)}, {(-1.0 / 100.0), (-0.3 / 100.0), (1.35 / 100.0)}, {(0.5 / 100.0), (-0.5 / 100.0), (0.95/ 100.0)} },   // Row 3



    // Row 4 (unchanged)
    { {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (1.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)} },  // Row 4

    // Row 5 (unchanged)
    { {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(-2.4 / 100.0), (0.0 / 100.0), (1.9 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (1.8 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (1.8 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (1.8 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)} },  // Row 5

    // Row 6 (unchanged)
    { {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.2 / 100.0)}, {(-1.9 / 100.0), (0.0 / 100.0), (2.7 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (4.2 / 100.0)}, {(-2.0 / 100.0), (0.0 / 100.0), (4.2 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)} },  // Row 6

    // Row 7 (unchanged)
    { {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)} },  // Row 7

    // Row 8 (unchanged)
    { {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)}, {(0.0 / 100.0), (0.0 / 100.0), (2.0 / 100.0)} }   // Row 8
};





        

};

#endif // ROBOTMANIPULATOR_H

