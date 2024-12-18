import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from pynput import keyboard

class CubeMover(Node):
    def __init__(self):
        super().__init__('cube_mover')
        self.publisher = self.create_publisher(Marker, 'cube_pose', 10)
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.step_size = 0.1  # Movement increment

        self.get_logger().info("Cube Mover initialized. Use arrow keys to move the cube.")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.position[1] += self.step_size
            elif key == keyboard.Key.down:
                self.position[1] -= self.step_size
            elif key == keyboard.Key.left:
                self.position[0] -= self.step_size
            elif key == keyboard.Key.right:
                self.position[0] += self.step_size
            elif key == keyboard.KeyCode.from_char('q'):
                self.get_logger().info("Exiting...")
                rclpy.shutdown()
                return

            self.publish_marker()
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "marker_mover"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the size of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set the color of the marker (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)
        self.get_logger().info(f"Published marker at position: {self.position}")


def main(args=None):
    rclpy.init(args=args)
    node = CubeMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
