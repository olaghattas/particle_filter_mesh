import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PointStamped
import tkinter as tk
import threading

class DoorSensorPublisher(Node):
    def __init__(self):
        super().__init__('door_sensor_publisher')

        # Publishers for each door sensor (True/False states)
        self.bathroom_pub = self.create_publisher(Bool, '/smartthings_sensors_door_bathroom', 10)
        self.bedroom_pub = self.create_publisher(Bool, '/smartthings_sensors_door_bedroom', 10)
        self.outdoor_pub = self.create_publisher(Bool, '/smartthings_sensors_door_outdoor', 10)

        # Publishers for geometry points (clicked_point data)
        self.kitchen_point_pub = self.create_publisher(Point, '/zed_kitchen', 10)
        self.lv_point_pub = self.create_publisher(Point, '/zed_living_room', 10)
        self.doorway_point_pub = self.create_publisher(Point, '/zed_doorway', 10)
        self.coor_point_pub = self.create_publisher(Point, '/zed_corridor', 10)

        # Subscriber for clicked_point topic
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        # State tracking for active point publisher
        self.active_point_publisher = None
        self.listening = False

    def publish_state(self, publisher, state: bool):
        # Publishes the True/False state repeatedly for robustness
        for count in range(30):
            msg = Bool()
            msg.data = state
            publisher.publish(msg)
            self.get_logger().info(f"Published {state} on {publisher.topic}")

    def set_active_point_publisher(self, publisher):
        """Sets or toggles the active publisher for the clicked_point data."""
        if self.active_point_publisher == publisher and self.listening:
            # If the same button is clicked again, stop listening
            self.listening = False
            point_msg = Point(x=0.0, y=0.0, z=1.0)  # Assuming 2D so z= 1.0 indcates no reading
            self.active_point_publisher.publish(point_msg)
            self.get_logger().info(f"Stopped listening to {publisher.topic}")
        else:
            # If a new button is clicked or listening is off, start listening
            self.active_point_publisher = publisher
            self.listening = True
            self.get_logger().info(f"Started listening to {publisher.topic}")

    def clicked_point_callback(self, msg: PointStamped):
        """Callback for /clicked_point topic, publishing x and y to the active point topic."""
        if self.listening and self.active_point_publisher:
            point_msg = Point(x=msg.point.x, y=msg.point.y, z=0.0)  # Assuming 2D
            self.active_point_publisher.publish(point_msg)
            self.get_logger().info(
                f"Published Point x: {point_msg.x}, y: {point_msg.y} to {self.active_point_publisher.topic}"
            )

class DoorSensorGUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Door Sensor Control")

        # Create the True/False buttons for each sensor
        self.create_sensor_controls("Bathroom", self.node.bathroom_pub)
        self.create_sensor_controls("Bedroom", self.node.bedroom_pub)
        self.create_sensor_controls("Outdoor", self.node.outdoor_pub)

        # Create the toggle point publisher buttons for each point
        self.create_point_publisher_button("Kitchen Point", self.node.kitchen_point_pub)
        self.create_point_publisher_button("Living Room Point", self.node.lv_point_pub)
        self.create_point_publisher_button("Doorway Point", self.node.doorway_point_pub)
        self.create_point_publisher_button("Corridor Point", self.node.coor_point_pub)

    def create_sensor_controls(self, name, publisher):
        # Label for each door sensor
        tk.Label(self.root, text=f"{name} Door Sensor:").pack()

        # True button
        true_button = tk.Button(self.root, text="True",
                                command=lambda: self.node.publish_state(publisher, True),
                                bg="green", width=10)
        true_button.pack(pady=5)

        # False button
        false_button = tk.Button(self.root, text="False",
                                 command=lambda: self.node.publish_state(publisher, False),
                                 bg="red", width=10)
        false_button.pack(pady=5)

    def create_point_publisher_button(self, name, publisher):
        """Creates a button to set or toggle the active publisher for clicked_point data."""
        button = tk.Button(self.root, text=name, width=20,
                           command=lambda: self.set_active_point_publisher(publisher, button))
        button.pack(pady=5)

    def set_active_point_publisher(self, publisher, button):
        """Toggles the active publisher for the clicked_point data."""
        self.node.set_active_point_publisher(publisher)
        self.toggle_button_color(button)

    def toggle_button_color(self, button):
        """Toggles the button color to indicate whether it's active or not."""
        if button['bg'] == 'lightblue':  # Active state
            button.config(bg='lightgray')  # Reset to original color
        else:
            button.config(bg='lightblue')  # Set to active color

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DoorSensorPublisher()
    gui = DoorSensorGUI(node)

    # Create a separate thread for running the ROS spin function
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    # Run the GUI in the main thread
    gui.run()

    # Once the GUI loop finishes, shutdown ROS
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
