#!/usr/bin/env python3.11

from pathlib import Path
import cv2
import depthai as dai
import time
import blobconverter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Constants
SHOW_PREVIEW = True  # Set to False to disable preview on the computer
SAVE_INTERVAL = 0.2  # Save image at 5 fps
IMG_MAP_PATH = '/tmp/img_camera.png'
DATA_FILE_PATH = '/tmp/person_track_data.txt'

print("FLAG TRACKER (CAMERA)")

class FlagTracker(Node):

    def __init__(self):
        super().__init__('flag_tracker')
        self.publisher_ = self.create_publisher(String, 'flagdata', 10)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.data_to_send = ""
        self.found = False  # Variable de estado

        # Argument parsing and pipeline creation
        self.declare_parameter('nnPath', '')
        self.declare_parameter('full_frame', False)

        nnPathDefault = blobconverter.from_zoo(
            name="mobilenet-ssd",
            shaves=5,
            zoo_type="intel"
        )

        self.nnPath = self.get_parameter('nnPath').get_parameter_value().string_value
        if not self.nnPath:
            self.nnPath = nnPathDefault

        self.fullFrameTracking = self.get_parameter('full_frame').get_parameter_value().bool_value

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()

    def setup_pipeline(self):
        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        objectTracker = self.pipeline.create(dai.node.ObjectTracker)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        trackerOut = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("preview")
        trackerOut.setStreamName("tracklets")

        # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Setting node configs
        stereo.initialConfig.setConfidenceThreshold(255)

        spatialDetectionNetwork.setBlobPath(self.nnPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        objectTracker.setDetectionLabelsToTrack([15])  # Track only person
        # Possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        # Take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        # LR-check is required for depth alignment
        stereo.setLeftRightCheck(True)
        # dai.RawStereoDepthConfig.AlgorithmControl.DepthAlign
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
        objectTracker.out.link(trackerOut.input)

        if self.fullFrameTracking:
            camRgb.setPreviewKeepAspectRatio(False)
            camRgb.video.link(objectTracker.inputTrackerFrame)
            objectTracker.inputTrackerFrame.setBlocking(False)
            # Do not block the pipeline if it's too slow on full frame
            objectTracker.inputTrackerFrame.setQueueSize(2)
        else:
            spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)

    def send_data(self, data):
        if data:
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.data_to_send = data

    def timer_callback(self):
        if self.data_to_send:
            self.publisher_.publish(String(data=self.data_to_send))

    def start_tracking(self):
        with dai.Device(self.pipeline) as device:
            preview = device.getOutputQueue("preview", 4, False)
            tracklets = device.getOutputQueue("tracklets", 4, False)

            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)
            last_save_time = time.monotonic()

            while rclpy.ok():
                imgFrame = preview.get()
                track = tracklets.get()

                counter += 1
                current_time = time.monotonic()
                if (current_time - startTime) > 1:
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                frame = imgFrame.getCvFrame()
                trackletsData = track.tracklets
                found_target = False
                data_to_publish = ""
                for t in trackletsData:
                    roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                    x1 = int(roi.topLeft().x)
                    y1 = int(roi.topLeft().y)
                    x2 = int(roi.bottomRight().x)
                    y2 = int(roi.bottomRight().y)

                    try:
                        label = labelMap[t.label]
                    except:
                        label = t.label

                    if t.status.name == "TRACKED":
                        if not self.found:
                            self.found = True
                        found_target = True
                        data_to_publish += f"X:{t.spatialCoordinates.x},Y:{t.spatialCoordinates.y},Z:{t.spatialCoordinates.z};"

                        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                        cv2.putText(frame, f"X: {int(t.spatialCoordinates.x) / 1000} m", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y) / 1000} m", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z) / 1000} m", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))

                        print("Tracked @: X: " + str(int(t.spatialCoordinates.x) / 1000) + "m | Y: " + str(int(t.spatialCoordinates.y) / 1000) + "m | Z: " + str(int(t.spatialCoordinates.z) / 1000) + "m")
                if found_target:
                    self.send_data(data_to_publish.rstrip(";"))
                else:
                    self.data_to_send = ""

                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

                # Save the frame to a file at 5 fps
                if current_time - last_save_time >= SAVE_INTERVAL:
                    cv2.imwrite(IMG_MAP_PATH, frame)
                    last_save_time = current_time

                if SHOW_PREVIEW:
                    cv2.imshow("tracker", frame)

                if cv2.waitKey(1) == 27:
                    break

def main(args=None):
    rclpy.init(args=args)
    flag_tracker = FlagTracker()
    flag_tracker.start_tracking()
    flag_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
