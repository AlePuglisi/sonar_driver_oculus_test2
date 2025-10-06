#!/usr/bin/env python3
import numpy as np
import cv2
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sonar_oculus_interface.msg import OculusPing
from sensor_msgs.msg import Image

REVERSE_Z = 1
bridge = CvBridge()

to_rad = lambda bearing: bearing * np.pi / 18000
vis_lines = True


class OculusViewer(Node):
    def __init__(self, model: str):
        super().__init__("oculus_viewer_" + model)

        # --- Declare ROS2 parameters (replacement for dynamic_reconfigure) ---
        self.declare_parameter("Raw", False)
        self.declare_parameter("Colormap", 1)

        # --- Internal state ---
        self.res = None
        self.height = None
        self.rows = None
        self.width = None
        self.cols = None
        self.map_x = None
        self.map_y = None
        self.f_bearings = None

        # --- Subscriber and Publisher ---
        ping_sub_topic = "/sonar_oculus_node/" + model + "/ping"
        self.ping_sub = self.create_subscription(
            OculusPing, ping_sub_topic, self.ping_callback, 10
        )
        self.img_pub = self.create_publisher(
            Image, "/sonar_oculus_node/" + model + "/image", 10
        )

        self.get_logger().info(f"Started Oculus Viewer for model {model}")

    # ---------------------------------------------------
    # Precompute mapping from polar sonar grid to image grid
    # ---------------------------------------------------
    def generate_map_xy(self, ping):
        # Extract sonar geometry parameters
        _res = ping.range_resolution
        _height = ping.num_ranges * _res
        _rows = ping.num_ranges
        _width = (
            np.sin(to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2
        )
        _cols = int(np.ceil(_width / _res))

        if (
            self.res == _res
            and self.height == _height
            and self.rows == _rows
            and self.width == _width
            and self.cols == _cols
        ):
            return

        self.res, self.height, self.rows, self.width, self.cols = (
            _res,
            _height,
            _rows,
            _width,
            _cols,
        )

        # Prepare the bearing (beamindex mapping)
        bearings = to_rad(np.asarray(ping.bearings, dtype=np.float32))
        self.f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind="linear",
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True,
        )

        # Generate cartesian coordinate grid 
        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        x = self.res * (self.rows - YY)
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        # Convert Cartesian coordinate to polar sonar coordinates 
        b = np.arctan2(y, x) * REVERSE_Z
        r = np.sqrt(np.square(x) + np.square(y))
        # Convert from phisical units to image units 
        self.map_y = np.asarray(r / self.res, dtype=np.float32)
        self.map_x = np.asarray(self.f_bearings(b), dtype=np.float32)

    # ---------------------------------------------------
    # Main callback for sonar pings
    # ---------------------------------------------------
    def ping_callback(self, msg: OculusPing):
        raw = self.get_parameter("Raw").value
        cm = self.get_parameter("Colormap").value

        # Decode raw sonar ping into numpy array
        img = np.frombuffer(msg.ping.data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        # Show directly polar image (raw image)
        if raw:
            # Normalize and Colorize for better visibility 
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            img = cv2.applyColorMap(img, cm)
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header = msg.header
            self.img_pub.publish(img_msg)
        # Process the image to transform in cartesian coordinates 
        else:
            self.generate_map_xy(msg)
            img = np.array(img, dtype=img.dtype, order="F")

            if self.cols > img.shape[1]:
                img.resize(self.rows, self.cols)
            # Overlays for visualization
            if vis_lines:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                cv2.line(img, (334, 0), (334, 1000), [0, 255, 0], 5)
                cv2.line(img, (177, 0), (177, 1000), [0, 255, 0], 5)

            img = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)
            img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
            img = cv2.applyColorMap(img, cm)

            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header = msg.header
            self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    if len(args) < 2:
        print("Please enter an argument for sonar model")
        print("Usage: ros2 run sonar_oculus oculus_viewer.py <M1200d|M750d>")
        return

    model = args[1]
    if model not in ["M1200d", "M750d", "M3000d"]:
        print("Invalid sonar model. Use M1200d or M750d or M3000d.")
        return

    node = OculusViewer(model)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys

    main(sys.argv)