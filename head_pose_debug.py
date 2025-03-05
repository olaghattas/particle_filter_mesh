import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from geometry_msgs.msg import TransformStamped, PointStamped, Point
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from tf_transformations import quaternion_from_matrix
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
import math
from zed_interfaces.msg import ObjectsStamped


class ZEDUSBPixelMapping(Node):
    def __init__(self):
        super().__init__('pixel_mapper')


        # Subscribe to the skeleton
        self.create_subscription(
            ObjectsStamped,
            '/zed_doorway/zed_node_doorway/body_trk/skeletons',
            self.objects_callback,
            10
        )



        self.u, self.v = 640, 200

    def objects_callback(self, msg: ObjectsStamped):
        print("skeleton")
        # print("skeleton", msg)
        # Iterate through each detected object in the message
        for obj in msg.objects:
            # Extract 2D and 3D bounding box information
            # head_bounding_box_2d = obj.head_bounding_box_2d.corners
            # head_bounding_box_3d = obj.head_bounding_box_3d.corners
            head_position = obj.head_position


            x, y, z = head_position[0], head_position[1], head_position[2]
            # self.publish_marker("zed_left_camera_frame", x, y, z, id=0, r=1.0)
            # transform = self.transform_point([x, y, z])

            # self._3d_to_pixel(*transform)

            print("head_pos x: ", x)
            print("head_pos y: ", y)
            print("head_pos z: ", z)




def main(args=None):
    rclpy.init(args=args)
    node = ZEDUSBPixelMapping()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()