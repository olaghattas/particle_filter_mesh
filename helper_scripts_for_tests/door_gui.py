import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk
import threading
import time  # Import standard time module for sleep

class DoorControlNode(Node):
    def __init__(self):
        super().__init__('door_control_node')

        # Create a publisher for the /smartthings_sensors_door_outdoor topic
        self.publisher = self.create_publisher(Bool, '/smartthings_sensors_door_outdoor', 10)

        # Create the GUI window
        self.window = tk.Tk()
        self.window.title("Door Control")

        # Create Open and Close buttons
        self.open_button = tk.Button(self.window, text="Open", command=self.open_door)
        self.open_button.pack(padx=20, pady=10)

        self.close_button = tk.Button(self.window, text="Close", command=self.close_door)
        self.close_button.pack(padx=20, pady=10)

        # Keep track of which button was pressed
        self.current_state = None

        # Flag to control continuous publishing
        self.continuous_publish_thread = None

    def open_door(self):
        if self.current_state != "open":
            # If close was clicked previously, reset close
            self.close_button.config(state="normal")

            # Set current state to "open"
            self.current_state = "open"
            self.get_logger().info("Door Opened")

            # Disable the open button once open is clicked
            self.open_button.config(state="disabled")

            # Start a new thread to continuously publish the open state (False)
            self.start_continuous_publishing(False)

    def close_door(self):
        if self.current_state != "close":
            # If open was clicked previously, reset open
            self.open_button.config(state="normal")

            # Set current state to "close"
            self.current_state = "close"
            self.get_logger().info("Door Closed")

            # Disable the close button once close is clicked
            self.close_button.config(state="disabled")

            # Start a new thread to continuously publish the close state (True)
            self.start_continuous_publishing(True)

    def start_continuous_publishing(self, is_close):
        # Stop any previous publishing thread
        if self.continuous_publish_thread is not None and self.continuous_publish_thread.is_alive():
            self.continuous_publish_thread.join()

        # Create a new thread to continuously publish the state
        self.continuous_publish_thread = threading.Thread(target=self.continuous_publish, args=(is_close,))
        self.continuous_publish_thread.daemon = True
        self.continuous_publish_thread.start()

    def continuous_publish(self, is_close):
        msg = Bool()
        msg.data = is_close  # True for "Close", False for "Open"
        while self.current_state == ("close" if is_close else "open"):
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing {'Close' if is_close else 'Open'} state")
            time.sleep(1.0)  # Publish every second using time.sleep()

    def run(self):
        # Start the Tkinter window main loop
        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)

    # Create and run the DoorControlNode
    door_control_node = DoorControlNode()
    door_control_node.run()

    # Spin the node to handle callbacks
    rclpy.spin(door_control_node)

    # Shutdown after the GUI window is closed
    door_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
