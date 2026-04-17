import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import time

# This node handles real-time object detection using the YOLO model.
# It captures video from the camera, processes each frame using the pretrained YOLOv8n COCO model,
# publishes a Bool flag indicating whether any detected object lies in the middle third of the frame,
# and publishes the annotated frame as a JPEG-compressed image so the MJPEG stream shows the
# annotated feed (replacing the standalone camera_publisher node).
# Publishers: 'obstacle_detected', 'camera/image/compressed'
# Subscribers: 'emergency_stop'
class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Loads the pretrained YOLOv8n model trained on COCO
        self.model = YOLO('yolov8n.pt')
        # Opens the webcam footage at index 0 (for the Pi) 
        self.cap = cv2.VideoCapture(0)
        # If webcame cannot open the node is shut down
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            self.destroy_node()
            return
        
        # Flag to control saving the video to a file
        # Set to True to enable video recording
        self.save_video = True

        # Flag to control saving annotated vs. unannotated video
        # Set to True to save the annotated video
        self.save_annotated = True

        # Cache the frame dimensions for the middle-third obstacle check
        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))

        # If video recording is enabled, set up the video writer
        # The video will be saved in the current directory labeled with the current timestamp as an .mp4 file (ignored by .gitignore)
        if self.save_video:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.video_filename = f"detection_{timestamp}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 5
            self.out = cv2.VideoWriter(self.video_filename, fourcc, fps, (self.frame_width, self.frame_height))

        # Publisher signaling whether an obstacle is in the middle third of the frame
        # Message type: Bool
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)

        # Publisher for the annotated camera frame (JPEG-compressed)
        # Message type: CompressedImage — replaces the standalone camera_publisher node
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        
        # Frame skip counter: run YOLO every 3rd frame (~3 Hz inference),
        # republish the last annotated frame on skipped frames (~10 Hz stream).
        self.frame_counter = 0
        self.skip_interval = 3
        self.last_annotated_frame = None

        # Timer to process the image every 0.1 seconds (artificially 10 FPS)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process_image)

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed and the camera will be released and recording saved
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

    # Publishes an annotated frame as a JPEG-compressed image
    def publish_frame(self, frame):
        encoded, jpeg = cv2.imencode('.jpg', frame)
        if encoded:
            image_msg = CompressedImage()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera'
            image_msg.format = 'jpeg'
            image_msg.data = jpeg.tobytes()
            self.image_pub.publish(image_msg)

    # Processes the image from the camera
    def process_image(self):
        # Read a frame from the camera
        ret, frame = self.cap.read()
        # If the frame is read successfully, process it
        if ret:
            # Flip the frame vertically (raw image is upside down due to camera orientation)
            frame = cv2.flip(frame, 0)

            self.frame_counter += 1

            # On skipped frames, republish the last annotated frame to keep the stream at ~10 Hz
            if self.frame_counter % self.skip_interval != 0:
                if self.last_annotated_frame is not None:
                    self.publish_frame(self.last_annotated_frame)
                return

            # Run YOLO inference (~3 Hz)
            results = self.model(frame, verbose=False)

            # Middle third of the frame horizontally — an obstacle here is in the boat's path
            left_bound = self.frame_width / 3
            right_bound = 2 * self.frame_width / 3

            # Check whether any detection with confidence > 0.5 has its bounding box center
            # inside the middle third of the frame
            obstacle_detected = False
            for box in results[0].boxes:
                if box.conf[0].item() <= 0.5:
                    continue
                x1, _, x2, _ = map(float, box.xyxy[0])
                cx = (x1 + x2) / 2
                if left_bound < cx < right_bound:
                    obstacle_detected = True
                    break

            # Publish the obstacle flag
            obstacle_msg = Bool()
            obstacle_msg.data = obstacle_detected
            self.obstacle_pub.publish(obstacle_msg)

            # Annotate the frame with bounding boxes and labels
            annotated_frame = results[0].plot()
            self.last_annotated_frame = annotated_frame

            # Publish the annotated frame
            self.publish_frame(annotated_frame)

            # If video recording is enabled, write the frame to the video file
            if self.save_video:
                if self.save_annotated:
                    self.out.write(annotated_frame)
                else:
                    self.out.write(frame)

    # Destroy the node when the emergency stop is triggered
    # Stops the camera and ends video recording if enabled
    def destroy_node(self,msg):
        if self.cap.isOpened():
            self.cap.release()
            if self.save_video:
                self.out.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    object_detector = ObjectDetector()
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        object_detector.get_logger().info("Shutting down camera")
    finally:
        object_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
