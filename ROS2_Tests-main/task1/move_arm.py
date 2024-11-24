import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from geometry_msgs.msg import Pose

class UR5MoveScript(Node):
    def __init__(self):
        super().__init__('ur5_move_script')
        
        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Define the sequence of positions
        self.positions = {
            "P1": [0.20, -0.47, 0.65],
            "P2": [0.75, 0.49, -0.05],
            "P3": [0.75, -0.23, -0.05],
            "D": [-0.69, 0.10, 0.44]
        }
        self.sequence = ["P1", "D", "P2", "D", "P3", "D"]

        # Move through the sequence
        self.move_sequence()

    def move_to_position(self, position):
        # Set up the target pose
        target_pose = Pose()
        target_pose.position.x = position[0]
        target_pose.position.y = position[1]
        target_pose.position.z = position[2]
        
        # Keep orientation fixed (pointing forward); adjust if needed
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        # Set and execute the pose target
        self.moveit2.move_to_pose(position=target_pose.position,quat_xyzw=target_pose.orientation, frame_id="base_link", tolerance_position=0.01, tolerance_orientation=0.01)
        self.moveit2.wait_until_executed(timeout_sec=10)  # Wait for execution to complete
        self.get_logger().info(f"Moved to position: {position}")

    def move_sequence(self):
        for pos in self.sequence:
            self.move_to_position(self.positions[pos])
            self.get_logger().info(f"Pausing briefly at position {pos}")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))  # Optional pause for observation

def main(args=None):
    rclpy.init(args=args)
    ur5_move_script = UR5MoveScript()
    rclpy.spin(ur5_move_script)
    ur5_move_script.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
