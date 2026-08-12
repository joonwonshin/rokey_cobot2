import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D
rclpy.init()
n = Node('check')
pub = n.create_publisher(CompressedImage, '/camera/camera/color/image_raw/compressed', 10)
got = []
n.create_subscription(Detection2D, '/detection2_d', lambda m: got.append(m), 10)
img = CompressedImage()
img.header.stamp.sec, img.header.stamp.nanosec = 1785760876, 123456789
img.header.frame_id = 'camera_color_optical_frame'
import time
t0 = time.time()
while time.time() - t0 < 8 and not got:
    pub.publish(img); rclpy.spin_once(n, timeout_sec=0.2)
assert got, "FAIL: /detection2_d 수신 0건"
d = got[0]
assert (d.header.stamp.sec, d.header.stamp.nanosec) == (1785760876, 123456789), f"stamp 불일치 {d.header.stamp}"
assert d.header.frame_id == 'camera_color_optical_frame'
print(f"PASS stamp={d.header.stamp.sec}.{d.header.stamp.nanosec} frame={d.header.frame_id} "
      f"center=({d.bbox.center.position.x},{d.bbox.center.position.y}) size=({d.bbox.size_x},{d.bbox.size_y})")
rclpy.shutdown()
