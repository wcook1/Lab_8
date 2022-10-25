#include "../include/scene_setup.hpp"

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "scene_setup");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

    double distance;
    double width;
    
    // Getting values from the launch file
    n.getParam("/scene_setup/distance", distance);
    n.getParam("/scene_setup/width", width);

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    std::string planning_frame = arm_move_group.getPlanningFrame();

    // Table
    moveit_msgs::CollisionObject robot_table_obj;
    std::string table_id = "table";
    shape_msgs::SolidPrimitive table_primitive;
    std::vector<double> table_dim = {1.12, 0.805, 0.766};
    std::string table_type = "BOX";
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.3125;
    table_pose.position.z = 0.383;
    ArmController::createCollisionObject(robot_table_obj, table_id, table_primitive, table_pose, table_type, table_dim, planning_frame);

    // Actual publishing
    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene_msg;
    ArmController::addCollisionObjectToScene(robot_table_obj, planning_scene_msg);

    planning_scene_msg.is_diff = true;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        planning_scene_diff_publisher.publish(planning_scene_msg);
        ROS_INFO("published");
        loop_rate.sleep();
    }
}