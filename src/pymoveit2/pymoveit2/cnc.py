#!/usr/bin/env python3
from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import astra as robot
from scipy.spatial.transform import Rotation as R

def main():
    rclpy.init()
    node = Node("arm_waypoint_follower")

    # Define waypoints (you can modify these positions as needed)
    waypoints = [
        [0.10, 0.10, 0.0],    # Point 1
        [0.10, -0.10, 0.0],   # Point 2
        [0.20, -0.10, 0.0],   # Point 3
        [0.20, 0.10, 0.0],   # Point 4
        [0.10, 0.10, 0.0]    # Return to start
    ]

    # Fixed orientation (same as arm_mover)
    roll, pitch, yaw = 0.0, 1.57, 0.0
    r = R.from_euler('xyz', [roll, pitch, yaw])
    quaternion = r.as_quat()

    # Declare parameters for cartesian planning
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("cartesian", True)
    node.declare_parameter("cartesian_max_step", 0.05)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Set the planner ID
    moveit2.planner_id = node.get_parameter("planner_id").get_parameter_value().string_value

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0
    # Get cartesian parameters
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    cartesian_fraction_threshold = node.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
    cartesian_jump_threshold = node.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
    cartesian_avoid_collisions = node.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value

    # Set cartesian parameters
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Move through each waypoint
    for i, position in enumerate(waypoints):
        node.get_logger().info(
            f"Moving to waypoint {i + 1}: {{position: {position}, quat_xyzw: {quaternion.tolist()}}}"
        )
        
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quaternion.tolist(),
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        
        # Wait for the movement to complete
        moveit2.wait_until_executed()
        
        # Optional: Add a small delay between waypoints
        # node.create_rate(1.0).sleep()

    node.get_logger().info("Waypoint sequence completed!")
    
    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
