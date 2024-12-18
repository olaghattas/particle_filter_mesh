import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PointStamped
import tkinter as tk
import threading
from visualization_msgs.msg import Marker


class DoorSensorPublisher(Node):
    def __init__(self):
        super().__init__('door_sensor_publisher')
        self.bathroom_pub = self.create_publisher(Bool, '/smartthings_sensors_door_bathroom', 10)
        self.bedroom_pub = self.create_publisher(Bool, '/smartthings_sensors_door_bedroom', 10)
        self.outdoor_pub = self.create_publisher(Bool, '/smartthings_sensors_door_outdoor', 10)
        self.kitchen_point_pub = self.create_publisher(Point, '/zed_kitchen', 10)
        self.lv_point_pub = self.create_publisher(Point, '/zed_living_room', 10)
        self.doorway_point_pub = self.create_publisher(Point, '/zed_doorway', 10)
        self.coor_point_pub = self.create_publisher(Point, '/zed_corridor', 10)
        # self.clicked_point_sub = self.create_subscription(
        #     PointStamped, '/clicked_point', self.clicked_point_callback, 10
        # )

        self.clicked_point_sub = self.create_subscription(
            Marker, '/cube_pose', self.clicked_point_callback, 10
        )
        self.active_point_publisher = None
        self.listening = False

    def publish_state(self, publisher, state: bool):
        for _ in range(30):
            msg = Bool()
            msg.data = state
            publisher.publish(msg)
            self.get_logger().info(f"Published {state} on {publisher.topic}")

    def set_active_point_publisher(self, publisher):
        if self.active_point_publisher == publisher and self.listening:
            self.listening = False
            point_msg = Point(x=0.0, y=0.0, z=1.0)
            self.active_point_publisher.publish(point_msg)
            self.active_point_publisher = None
            self.get_logger().info(f"Stopped listening to {publisher.topic}")
        else:
            self.active_point_publisher = publisher
            self.listening = True
            self.get_logger().info(f"Started listening to {publisher.topic}")

    def clicked_point_callback(self, msg: PointStamped):
        if self.listening and self.active_point_publisher:
            # print("x", msg.pose.position.x, "y",  msg.pose.position.y)
            point_msg = Point(x= msg.pose.position.x, y=msg.pose.position.y, z=0.0)
            self.active_point_publisher.publish(point_msg)
            self.get_logger().info(
                f"Published Point x: {point_msg.x}, y: {point_msg.y} to {self.active_point_publisher.topic}"
            )

class DoorSensorGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Door Sensor Control")
        self.active_button = None  # Track the currently active button
        self.create_sensor_controls("Bathroom", self.node.bathroom_pub)
        self.create_sensor_controls("Bedroom", self.node.bedroom_pub)
        self.create_sensor_controls("Outdoor", self.node.outdoor_pub)
        self.create_point_publisher_button("Kitchen Point", self.node.kitchen_point_pub)
        self.create_point_publisher_button("Living Room Point", self.node.lv_point_pub)
        self.create_point_publisher_button("Doorway Point", self.node.doorway_point_pub)
        self.create_point_publisher_button("Corridor Point", self.node.coor_point_pub)

    def create_sensor_controls(self, name, publisher):
        tk.Label(self.root, text=f"{name} Door Sensor:").pack()
        true_button = tk.Button(self.root, text="True",
                                command=lambda: self.node.publish_state(publisher, True),
                                bg="green", width=10)
        true_button.pack(pady=5)
        false_button = tk.Button(self.root, text="False",
                                 command=lambda: self.node.publish_state(publisher, False),
                                 bg="red", width=10)
        false_button.pack(pady=5)

    def create_point_publisher_button(self, name, publisher):
        button = tk.Button(self.root, text=name, width=20,
                           command=lambda: self.set_active_point_publisher(publisher, button))
        button.pack(pady=5)

    def set_active_point_publisher(self, publisher, button):
        if self.active_button == button:
            self.node.set_active_point_publisher(publisher)
            self.toggle_button_color(button, deactivate=True)
            self.active_button = None
        else:
            if self.active_button:
                self.toggle_button_color(self.active_button, deactivate=True)
            self.node.set_active_point_publisher(publisher)
            self.toggle_button_color(button)
            self.active_button = button

    def toggle_button_color(self, button, deactivate=False):
        if deactivate:
            button.config(bg='lightgray')
        else:
            button.config(bg='lightblue')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DoorSensorPublisher()
    gui = DoorSensorGUI(node)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    gui.run()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
