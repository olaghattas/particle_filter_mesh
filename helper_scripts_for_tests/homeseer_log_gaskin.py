from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import requests
from datetime import datetime

class HomeSeerPublisher(Node):
    def __init__(self):
        super().__init__('homeseer_publisher')
        self.url = "http://192.168.50.85/json?request=getstatus"

        ## 22 open 23 closed (doors)
        ## 8 motion detected (motion sensors)

        # Mapping door names to reference IDs
        self.sensor_refs = {
            "main_door": 20, #ds3
            "bedroom_door": 15, # ds2
            "trash_door": 10, # ds1
            "back_door": 25, # ds4
            "motion_bedroom": 39, # ms2
            "motion_trash": 29, #ms1
            "motion_main": 44 # ms3
        }

        self.states = {
            "main_door": False,
            "bedroom_door": False,
            "trash_door": False,
            "back_door": False,
            "motion_bedroom": False,
            "motion_trash": False,
            "motion_main": False
        }
        self.prev_states = self.states.copy()

        # Create ROS publishers
        self.sensor_publishers = {
            "main_door": self.create_publisher(Bool, 'sensors_main_door', 10),
            "bedroom_door": self.create_publisher(Bool, 'sensors_bedroom_door', 10),
            "trash_door": self.create_publisher(Bool, 'sensors_trash_door', 10),
            "back_door": self.create_publisher(Bool, 'sensors_back_door', 10),
            "motion_bedroom": self.create_publisher(Bool, 'sensors_motion_bedroom', 10),
            "motion_trash": self.create_publisher(Bool, 'sensors_motion_trash', 10),
            "motion_main": self.create_publisher(Bool, 'sensors_motion_main', 10)

        }

        update_period = 2 # sec
        self.timer = self.create_timer(update_period, self.check_doors)

    def log_change(self, sensor_name: str, new_value: bool):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        if "door" in sensor_name:
            status = "closed" if new_value else "open"
        else:  # motion sensors
            status = "motion detected" if new_value else "no motion"

        log_line = f"{now} - Changed to: {status}\n"
        filename = f"{sensor_name}.log"
        print(f"sensor {sensor_name}: {now} - Changed to: {status}")
        with open(filename, "a") as f:
            f.write(log_line)


    def check_doors(self):

        try:
            response = requests.get(self.url)
            data = response.json()
        except Exception as e:
            print(f"Failed to get sensor data: {e}")
            return


        devices = data.get("Devices", [])
        for device in devices:

            ref = device.get("ref")
            value = device.get("value")  # Extract the numeric status


            for sensor_name, sensor_ref in self.sensor_refs.items():
                if ref == sensor_ref:
                    if "door" in sensor_name:
                        current_val = (value == 23)
                    else:  # motion sensors
                        current_val = (value == 8)

                    if self.prev_states[sensor_name] != current_val:
                        self.log_change(sensor_name, current_val)
                        self.prev_states[sensor_name] = current_val

                    # Publish updated state
                    msg = Bool()
                    msg.data = current_val
                    self.sensor_publishers[sensor_name].publish(msg)



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