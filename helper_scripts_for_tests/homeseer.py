from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import requests

class HomeSeerPublisher(Node):
    def __init__(self):
        super().__init__('homeseer_publisher')
        self.url = "http://192.168.50.180/json?request=getstatus"

        # Mapping door names to reference IDs
        self.sensor_refs = {
            "main_door": 20,
            "bedroom_door": 25,
            "atelier_door": 30,
            "motion_bedroom": 9, # ms1
            "motion_corridor": 14 # ms 2
        }

        # Initialize door states
        self.main_door_state = Bool()
        self.bedroom_door_state = Bool()
        self.atelier_door_state = Bool()
        self.bedroom_motion_detection = Bool()
        self.corridor_motion_detection = Bool()

        # Create ROS publishers
        self.publisher_main_door = self.create_publisher(Bool, 'sensors_main_door', 10)
        self.publisher_bedroom_door = self.create_publisher(Bool, 'sensors_bedroom_door', 10)
        self.publisher_atelier_door = self.create_publisher(Bool, 'sensors_atelier_door', 10)

        self.publisher_motion_bedroom = self.create_publisher(Bool, 'sensors_motion_bedroom', 10)
        self.publisher_motion_corridor = self.create_publisher(Bool, 'sensors_motion_corridor', 10)

        update_period = 2 # sec
        self.timer = self.create_timer(update_period, self.check_doors)

    def check_doors(self):
        response = requests.get(self.url)
        data = response.json()

        devices = data.get("Devices", [])
        for device in devices:
            ref = device.get("ref")
            value = device.get("value")  # Extract the numeric status

            # Check which door sensor matches the `ref` ID
            if ref == self.sensor_refs["main_door"]:
                self.main_door_state.data =  (int(value) == 23)
                print(f'Door sensor {ref}: Value = {value}, Published: {self.main_door_state.data}')

            elif ref == self.sensor_refs["bedroom_door"]:
                self.bedroom_door_state.data = (int(value) == 23)
                print(f'Door sensor {ref}: Value = {value}, Published: {self.bedroom_door_state.data}')

            elif ref == self.sensor_refs["atelier_door"]:
                self.atelier_door_state.data = (int(value) == 23)
                print(f'Door sensor {ref}: Value = {value}, Published: {self.atelier_door_state.data}')

            if ref == self.sensor_refs["motion_bedroom"]:
                self.bedroom_motion_detection.data =  (int(value) == 8)
                print(f'Motion sensor {ref}: Value = {value}, Published: {self.bedroom_motion_detection.data}')

            elif ref == self.sensor_refs["motion_corridor"]:
                self.corridor_motion_detection.data = (int(value) == 8)
                print(f'Motion sensor {ref}: Value = {value}, Published: {self.corridor_motion_detection.data}')


            self.publisher_bedroom_door.publish(self.bedroom_door_state)
            self.publisher_atelier_door.publish(self.atelier_door_state)
            self.publisher_main_door.publish(self.main_door_state)
            self.publisher_motion_bedroom .publish(self.bedroom_motion_detection)
            self.publisher_motion_corridor.publish(self.corridor_motion_detection)



def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = HomeSeerPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()