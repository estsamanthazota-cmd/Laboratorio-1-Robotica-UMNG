import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class RobotMover(Node):

    def __init__(self):
        super().__init__('robot_mover')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )

    def move_robot(self):

        positions = [
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.5, 0.5, 0.0],
            [0.5, 0.5, 0.5],
            [0.0, 0.0, 0.0]
        ]

        for pos in positions:

            msg = Float64MultiArray()
            msg.data = pos

            self.publisher.publish(msg)

            self.get_logger().info(f"Moving to {pos}")

            time.sleep(3)


def main(args=None):

    rclpy.init(args=args)

    mover = RobotMover()

    mover.move_robot()

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
