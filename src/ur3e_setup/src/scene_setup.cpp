#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "scene_setup");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    double dist_to_wall;
    
    // Getting values from the launch file
    n.getParam("/scene_setup/distance", dist_to_wall);


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface("manipulator");

    moveit_msgs::CollisionObject collision_object;

    ROS_INFO("Preparing collision object");    
    collision_object.id = "box1";
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.12;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 1.3;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = -dist_to_wall;
    box_pose.position.z = 0.65;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO("Adding an object into the world");
    sleep(2.0);
    planning_scene_interface.applyCollisionObjects(collision_objects);
    sleep(2.0);    

    ros::shutdown();    
    return 0;
}