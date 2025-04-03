#!/usr/bin/python3
import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from threading import Lock
from vision_msgs.msg import Detection2DArray, Detection2D

class VisualizationNode:
    """
    A ROS (Robot Operating System) node for visualizing object detection results on images.
    This node subscribes to image and detection topics, processes the data, and overlays bounding boxes
    and labels on the images to display detection results in real-time.

    Attributes:
    - _br (CvBridge): Converts ROS image messages to OpenCV format.
    - _image (np.array): Stores the current image for visualization.
    - _image_lock (Lock): Ensures thread-safe access to the image data.

    Methods:
    - __init__(self): Initializes the node and its attributes.
    - __call__(self): Starts the ROS node, subscribes to topics, and displays visualized results.
    - __receive_image(self, msg: Image): Callback to handle incoming raw image messages.
    - __receive_detection(self, msg: Detection2DArray): Callback to handle incoming detection messages.
    - __draw(self, d: Detection2D): Draws bounding boxes and labels on the current image.
    """
    
    def __init__(self) -> None:
        """
        Initializes the VisualizationNode class, setting up necessary attributes for image processing.

        Attributes:
        - _br (CvBridge): Converts between ROS and OpenCV image formats.
        - _image (np.array): Stores the current image for visualization.
        - _image_lock (Lock): Ensures thread-safe access to the image data.
        """
        self._br = CvBridge()
        self._image = None
        self._image_lock = Lock()

    def __call__(self) -> None:
        """
        Initializes the ROS node, subscribes to image and detection topics, and starts visualizing results.

        Subscribes to:
        - "/image_feed": For raw image data.
        - "/face_detection": For object detection data.

        Displays the processed images with bounding boxes and labels.
        """
        rospy.init_node('visualization_node')
        rospy.Subscriber("/image_feed", Image, self.__receive_image)
        rospy.Subscriber("/face_detection", Detection2DArray, self.__receive_detection)

        try:
            while not rospy.is_shutdown():
                with self._image_lock:
                    if self._image is not None:
                        cv2.imshow("Object Visualization", self._image)
                        cv2.waitKey(delay=1)
        except KeyboardInterrupt:
            rospy.loginfo("Visualization Node shutting down.")

    def __receive_image(self, msg: Image) -> None:
        """
        Callback function for receiving raw images.

        Args:
        - msg (Image): Incoming image message from the subscribed topic.

        Converts the received ROS image message to an OpenCV format and updates the current image.
        """
        with self._image_lock:
            self._image = self._br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def __receive_detection(self, msg: Detection2DArray) -> None:
        """
        Callback function for receiving object detection messages.

        Args:
        - msg (Detection2DArray): Incoming object detection data from the subscribed topic.

        Iterates through the detection results and overlays bounding boxes and labels on the image.
        """
        with self._image_lock:
            for d in msg.detections:
                self.__draw(d)

    def __draw(self, d: Detection2D) -> None:
        """
        Draws bounding boxes and labels on the image for a single detection.

        Args:
        - d (Detection2D): Object detection data containing bounding box and label information.

        The method calculates bounding box coordinates, assigns colors based on object ID, and overlays
        the information on the current image.
        """
        c = "Person" if d.results[0].id == 1 else "Face" if d.results[0].id == 2 else "Error Id"
        s = d.results[0].score
        b = d.bbox

        if self._image is None:
            rospy.logwarn("No image available for visualization.")
            return

        image = self._image.copy()
        h, w, _ = image.shape

        # Calculate bounding box coordinates
        x1 = int((b.center.x - b.size_x / 2) * w)
        y1 = int((b.center.y - b.size_y / 2) * h)
        x2 = int((b.center.x + b.size_x / 2) * w)
        y2 = int((b.center.y + b.size_y / 2) * h)

        # Define color based on object ID
        color = (255, 0, 0) if d.results[0].id == 1 else (0, 255, 0) if d.results[0].id == 2 else (0, 0, 255)

        # Draw bounding box and label on the image
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        cv2.putText(image, f"{c} {s:.2f}", (x1 - 9, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)

        self._image = image

if __name__ == "__main__":
    visualization_node = VisualizationNode()
    visualization_node()