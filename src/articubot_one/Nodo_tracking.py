import cv2
import depthai as dai
import blobconverter
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from custom_msgs.msg import TrackedObject
from colored import fore, style
import time

# Constants
SHOW_PREVIEW = True
SAVE_INTERVAL = 0.2
IMG_MAP_PATH = '/tmp/img_camera.png'
DATA_FILE_PATH = '/tmp/person_and_object_track_data.txt'

class PersonAndObjectTracker(Node):
    def __init__(self):
        super().__init__('person_and_object_tracker')
        self.publisher_ = self.create_publisher(TrackedObject, 'person_data', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.data_to_send = None
        self.last_publish_time = self.get_clock().now()
        self.publish_interval = 0.33

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

        self.pipeline = dai.Pipeline()
        self.setup_pipeline()


    def setup_pipeline(self):
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

        camRgb.setPreviewSize(300, 300)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo.initialConfig.setConfidenceThreshold(255)

        spatialDetectionNetwork.setBlobPath(self.nnPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        objectTracker.setDetectionLabelsToTrack([15])  # Track only person
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
        objectTracker.out.link(trackerOut.input)

        if self.fullFrameTracking:
            camRgb.setPreviewKeepAspectRatio(False)
            camRgb.video.link(objectTracker.inputTrackerFrame)
            objectTracker.inputTrackerFrame.setBlocking(False)
            objectTracker.inputTrackerFrame.setQueueSize(2)
        else:
            spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)


    def send_data(self, id, point):
        if point:
            msg = TrackedObject()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            msg.id = UInt32(data=id)
            msg.point.x = point[0]
            msg.point.y = point[1]
            msg.point.z = point[2]
            self.publisher_.publish(msg)


    def timer_callback(self):
        if self.data_to_send:
            id, (x, y, z) = self.data_to_send
            self.send_data(id, (x, y, z))


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
                frame = imgFrame.getCvFrame()
                trackletsData = track.tracklets

                counter += 1
                current_time = time.monotonic()
                if (current_time - startTime) > 1:
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                # Detectar el objeto amarillo en la imagen
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (25, 100, 100), (35, 255, 255))  # Ajustar rango para amarillo
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                yellow_object_position = None
                for contour in contours:
                    if cv2.contourArea(contour) > 50:  # Filtrar objetos pequeños
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            # Calcular el centroide
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            yellow_object_position = (cX, cY)

                            print(f"{fore.LIGHT_YELLOW}Yellow Tracked @ X: {cX:.2f} | Y: {cY:.2f} {style.RESET}")

                            # Dibujar el objeto amarillo en la imagen
                            cv2.circle(frame, (cX, cY), 5, color, 2)
                            cv2.putText(frame, f"Yellow @ ({cX}, {cY})",
                                        (cX + 10, cY - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 0), 2)

                # Procesar las detecciones de tracklets
                for t in trackletsData:
                    if t.status.name == "TRACKED":
                        # Obtener coordenadas 3D del objeto detectado
                        id = t.id
                        x = t.spatialCoordinates.x / 1000  # Convertir a metros
                        y = t.spatialCoordinates.y / 1000
                        z = t.spatialCoordinates.z / 1000

                        # Dibujar información en el frame
                        roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                        x1, y1, x2, y2 = int(roi.topLeft().x), int(roi.topLeft().y), int(roi.bottomRight().x), int(roi.bottomRight().y)

                        thing = "Person"

                        print(f"{fore.LIGHT_CYAN}Person Tracked @ ID: {t.id} | X: {x:.2f} m | Y: {y:.2f} m | Z: {z:.2f} m{style.RESET}")

                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, f"{thing} ID: {id}", (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"Status: {t.status.name}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 0))
                        cv2.putText(frame, f"X: {x:.2f} m", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"Y: {y:.2f} m", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))
                        cv2.putText(frame, f"Z: {z:.2f} m", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.45, (255, 255, 0))

                        # Comparar con el objeto amarillo si se detectó
                        if yellow_object_position:
                            cX, cY = yellow_object_position
                            if x1 <= cX <= x2 and y1 <= cY <= y2:

                                # Publicar si el objeto amarillo está dentro de la caja de la persona y se acabó el cooldown
                                time_now = self.get_clock().now()
                                elapsed_time = (time_now - self.last_publish_time).nanoseconds / 1e9  # Convertir a segundos
                                if elapsed_time >= self.publish_interval:
                                    self.send_data(id, (x, y, z))
                                    self.last_publish_time = time_now
                
                # Mostrar FPS en el frame
                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color) #(255, 255, 255), 1

                # Mostrar las imágenes para depuración
                if SHOW_PREVIEW:
                    cv2.imshow("Preview", frame)
                    cv2.imshow("Yellow Mask", mask)

                if cv2.waitKey(1) == 27:  # Presiona ESC para salir
                    break


def main(args=None):
    rclpy.init(args=args)
    tracker = PersonAndObjectTracker()
    tracker.start_tracking()
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

