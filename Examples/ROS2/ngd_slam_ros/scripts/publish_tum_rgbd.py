#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class TumRgbdPublisher(Node):
    def __init__(
        self,
        sequence_dir: Path,
        association_file: Path,
        rgb_topic: str,
        depth_topic: str,
        fps: float,
        max_frames: int,
    ) -> None:
        super().__init__("tum_rgbd_publisher")
        self.sequence_dir = sequence_dir
        self.association_file = association_file
        self.rgb_pub = self.create_publisher(Image, rgb_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        self.fps = fps
        self.max_frames = max_frames

    def _make_rgb_msg(self, image, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        return msg

    def _make_depth_msg(self, image, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = image.tobytes()
        return msg

    def _load_pairs(self):
        pairs = []
        with self.association_file.open() as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                pairs.append((self.sequence_dir / parts[1], self.sequence_dir / parts[3]))
        return pairs

    def publish(self) -> int:
        pairs = self._load_pairs()
        if self.max_frames > 0:
            pairs = pairs[: self.max_frames]

        for rgb_path, depth_path in pairs:
            rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            if rgb_bgr is None or depth is None:
                raise RuntimeError(f"failed to load frame pair: {rgb_path} {depth_path}")

            rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
            stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(self._make_rgb_msg(rgb, stamp))
            self.depth_pub.publish(self._make_depth_msg(depth, stamp))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.fps)

        self.get_logger().info(f"published_frames={len(pairs)}")
        return len(pairs)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish TUM RGB-D frames as ROS2 image topics.")
    parser.add_argument("sequence_dir", type=Path, help="TUM sequence root directory")
    parser.add_argument("association_file", type=Path, help="association txt file")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--max-frames", type=int, default=120)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TumRgbdPublisher(
        args.sequence_dir,
        args.association_file,
        args.rgb_topic,
        args.depth_topic,
        args.fps,
        args.max_frames,
    )
    try:
        node.publish()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
