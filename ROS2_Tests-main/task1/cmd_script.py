#!/usr/bin python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration



def main():
    rclpy.init()

    navigator = BasicNavigator()

    # # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 1.921
    # initial_pose.pose.position.y = -8.640
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = -3.140
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses to follow
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation.w = 3.14
    goal_pose1.pose.orientation.z = 0.0
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.86
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation.w = 0.97
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.84
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation.w = 2.78
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

            # # Some follow waypoints request change to demo preemption
            # if now - nav_start > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = now.to_msg()
            #     goal_pose4.pose.position.x = 0.0
            #     goal_pose4.pose.position.y = 0.0
            #     goal_pose4.pose.orientation.w = 1.0
            #     goal_pose4.pose.orientation.z = 0.0
            #     goal_poses = [goal_pose4]
            #     nav_start = now
            #     navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()