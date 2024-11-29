import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import depthai as dai


class OakDPublisher(Node):
    def __init__(self):

        super().__init__('oakd_publisher')

        # Publishers for RGB and Depth images
        # self.rgb_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_rect', 10)

        # Publishers for Camera Info
        # self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.bridge = CvBridge()
        
        # Create pipeline   
        pipeline = dai.Pipeline()

        # Define sources and outputs
        # cam_rgb = pipeline.createColorCamera()
        # cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        # cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # cam_rgb.setInterleaved(False)
        # cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        # cam_rgb.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        # xout_rgb = pipeline.createXLinkOut()
        # xout_rgb.setStreamName("rgb")
        # cam_rgb.video.link(xout_rgb.input)

        stereo = pipeline.createStereoDepth()
        stereo.setConfidenceThreshold(200)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(False)
        stereo.setRectifyEdgeFillColor(0)  # Black, to better see the borders

        cam_left = pipeline.createMonoCamera()
        cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_left.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        cam_right = pipeline.createMonoCamera()
        cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_right.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        cam_left.out.link(stereo.left)
        cam_right.out.link(stereo.right)

        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        # self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.get_logger().info('La cámara inició bacanamente papa')

        self.timer = self.create_timer(0.2, self.publish_frames)

    def publish_frames(self):
        # in_rgb = self.q_rgb.tryGet()
        in_depth = self.q_depth.tryGet()

        # if in_rgb is not None:
            # frame_rgb = in_rgb.getCvFrame()
            # rgb_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
            # self.rgb_pub.publish(rgb_msg)

            # rgb_msg.header.stamp = self.get_clock().now().to_msg()
            # self.rgb_info_pub.publish(self.get_camera_info('rgb', frame_rgb.shape[1], frame_rgb.shape[0]))

        if in_depth is not None:
            frame_depth = in_depth.getCvFrame()
            frame_depth = cv2.normalize(frame_depth, None, 0, 65535, cv2.NORM_MINMAX, cv2.CV_16UC1)
            depth_msg = self.bridge.cv2_to_imgmsg(frame_depth, encoding="16UC1")
            self.depth_pub.publish(depth_msg)

            depth_msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_info_pub.publish(self.get_camera_info('depth', frame_depth.shape[1], frame_depth.shape[0]))

    
    def get_camera_info(self, camera_name, width, height):
        """Create CameraInfo message with static values"""
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.width = width
        camera_info_msg.height = height
        camera_info_msg.distortion_model = "rational_polynomial"
        
        # Distortion coefficients (d) from your example
        camera_info_msg.d = [-2.5412163734436035, -3.7207038402557373, -3.240533987991512e-05,
                             -0.0007830002577975392, 23.312896728515625, -2.65240216255188,
                             -3.2185001373291016, 22.551231384277344]

        # Intrinsic matrix (K) from your example
        camera_info_msg.k = [999.761474609375, 0.0, 656.109130859375,
                             0.0, 999.6922607421875, 353.9346618652344,
                             0.0, 0.0, 1.0]

        # Rectification matrix (R)
        camera_info_msg.r = [1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0]

        # Projection matrix (P) from your example
        camera_info_msg.p = [999.761474609375, 0.0, 656.109130859375, 75.00000476837158,
                             0.0, 999.6922607421875, 353.9346618652344, 0.0,
                             0.0, 0.0, 1.0, 0.0]

        return camera_info_msg

def main(args=None):
    rclpy.init(args=args)
    node = OakDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
