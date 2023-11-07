#pragma once

#include <moveit_wrapper/moveit_imports.hpp>

/** @namespace ArmController arm_controller.hpp
 * "include/moveit_wrapper/arm_controller.hpp"
 *  @brief Utility functions for planning and execution routines to control the
 * UR3e
 */
namespace ArmController
{
    /**
     * @enum ArmController::SHAPE_PRIMITIVES
     * @brief Primitive shapes used for RViz objects
     */
    enum struct SHAPE_PRIMITIVES { BOX, CYLINDER, CONE, SPHERE };


    /**
     * @brief Construct random target pose for a given planning group
     * @param move_group_interface The MoveGroup to generate pose for
     * @param frame_name The name of the reference frame
     * @return The generated random pose
     */
    geometry_msgs::Pose getRandomPose(
        moveit::planning_interface::MoveGroupInterface &move_group_interface,
        std::string &frame_name);

    /**
     * @brief Construct plan to a given "named" target, these are obtained from
     * predefined RViz names
     * @param options The shadow_planning::PlanningOptions object for computing the
     * plan
     * @param move_group_interface The MoveGroup for which to plan
     * @param target_name The named target
     * @param plan The reference to the plan, which will be populated
     * @return Planning success or failure boolean
     */
    bool planToNamedTarget(
        MoveitPlanning::PlanningOptions &options,
        moveit::planning_interface::MoveGroupInterface &move_group_interface,
        std::string &target_name,
        moveit::planning_interface::MoveGroupInterface::Plan &plan);

    /**
     * @brief Construct plan to a given Pose target
     * @param options The shadow_planning::PlanningOptions object for computing the
     * plan
     * @param move_group_interface The MoveGroup for which to plan
     * @param target_pose The target pose
     * @param plan The reference to the plan, which will be populated
     * @return Planning success or failure boolean
     */
    bool planToPoseTarget(
        MoveitPlanning::PlanningOptions &options,
        moveit::planning_interface::MoveGroupInterface &move_group_interface,
        geometry_msgs::Pose &target_pose, std::string &reference_frame, moveit::planning_interface::MoveGroupInterface::Plan &plan);

    /**
     * @brief Construct plan to given Joint target
     * @param options shadow_planning::PlanningOptions object for computing the
     * plan
     * @param move_group_interface The MoveGroup for which to plan
     * @param plan The reference to the plan, which will be populated
     * @param joint_targets Map from joint name to the target value
     * @return Planning success or failure boolean
     */
    bool planToJointTargets(
        MoveitPlanning::PlanningOptions &options,
        moveit::planning_interface::MoveGroupInterface &move_group_interface,
        moveit::planning_interface::MoveGroupInterface::Plan &plan,
        std::map<std::string, double> &joint_targets);
    
    /**
     * @brief Create collision object for the planning scene, used by MoveIt
     * planning
     * @param collision_obj The MoveIt collision object
     * @param primitive_id The ID of the collision object
     * @param primitive The primitive used by the collision object
     * @param obj_pose The pose of the collision object
     * @param primitive_type The shape of the collision object
     * @param primitive_dims The dimension of the collision object
     */
    void createCollisionObject(
        moveit_msgs::CollisionObject &collision_obj, std::string primitive_id,
        shape_msgs::SolidPrimitive &primitive, geometry_msgs::Pose &obj_pose,
        std::string primitive_type,
        //ArmController::SHAPE_PRIMITIVES primitive_type,
        std::vector<double> &primitive_dim,
        std::string planning_frame);
    
    /**
     * @brief Add a collision object to the planning scene
     * @param collision_obj The object to add to scene
     * @param planning_scene_msg The PlanningScene to add object to
     * @param planning_scene_diff_publisher The publisher for updating the scene
     */
    void addCollisionObjectToScene(
        moveit_msgs::CollisionObject &collision_obj, moveit_msgs::PlanningScene &planning_scene_msg);
    
    /**
     * @brief plan a cartesian path from start_pose to end_pose
     * 
     */
    moveit_msgs::RobotTrajectory planCartesianPath(geometry_msgs::Pose start_pose, std::vector<geometry_msgs::Pose> waypoints, 
        std::string &reference_frame, moveit::planning_interface::MoveGroupInterface &move_group_interface);

    void close_gripper(ros::NodeHandle *nh);

    void open_gripper(ros::NodeHandle *nh);
};