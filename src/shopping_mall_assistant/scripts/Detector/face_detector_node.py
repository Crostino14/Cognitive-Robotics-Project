#!/usr/bin/python3
import cv2
import os
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from threading import Lock
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

script_dir = os.path.dirname(os.path.abspath(__file__))
FACE_PROTO = os.path.join(script_dir, "FaceDetectorModels", "opencv_face_detector.pbtxt")
FACE_MODEL = os.path.join(script_dir, "FaceDetectorModels", "opencv_face_detector_uint8.pb")
FACE_SCORE_TRESHOLD = 0.5

class FaceDetectorNode():
    """
    A ROS (Robot Operating System) node for detecting faces in images using a pre-trained FaceNet model.
    This node processes incoming image messages, detects faces, and publishes the results to a ROS topic.

    Attributes:
    - _br (CvBridge): Converts ROS image messages to OpenCV format.
    - _faceNet (cv2.dnn.Net): Pre-trained FaceNet model loaded for face detection.
    - _publisher (rospy.Publisher): Publishes face detection results as Detection2DArray messages.
    - _image_lock (Lock): Ensures thread-safe access to shared image data.

    Methods:
    - __init__(self): Initializes the node by loading the FaceNet model and setting up the publisher.
    - __call__(self): Starts the ROS node, subscribes to the image topic, and processes incoming images.
    - __handle_detector(self, msg: Image) -> Bool: Callback for handling image messages and performing face detection.
    """
    
    def __init__(self) -> None: 
        """
        Initializes the FaceDetectorNode class.

        Loads the FaceNet model, sets up a ROS publisher for detection results, and initializes an image lock.
        """
        self._br = CvBridge()
        self._faceNet = cv2.dnn.readNet(FACE_PROTO, FACE_MODEL)
        self._publisher = rospy.Publisher('/face_detection', Detection2DArray, queue_size=1)
        self._image_lock = Lock()
    
    def __call__(self) -> None:
        """
        Starts the ROS node, subscribes to the '/image_feed' topic, and processes images for face detection.

        This method continuously listens for incoming image messages and performs face detection using the FaceNet model.
        """
        rospy.init_node('face_detector', anonymous=True)
        rospy.Subscriber('/image_feed', Image, self.__handle_detector)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Face Detector Node shutting down.")

    def __handle_detector(self, msg: Image) -> Bool:
        """
        Callback function for processing incoming image messages and detecting faces.

        Args:
        - msg (Image): Incoming image message to be processed.

        Returns:
        - Bool: True if a face is detected; otherwise, False.

        This method processes the image, detects faces using the FaceNet model, and publishes the detection results.
        """
        frame = self._br.imgmsg_to_cv2(msg)     
        image = frame.copy() if frame is not None else None

        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], swapRB=True, crop=False)

        with self._image_lock:
            self._faceNet.setInput(blob)
            detections = self._faceNet.forward()
        
        message = Detection2DArray()
        
        for i in range(detections.shape[2]):
            score = detections[0, 0, i, 2]
            if score > FACE_SCORE_TRESHOLD and detections[0, 0, i, 5] < 1 and detections[0, 0, i, 6] < 1:
                xmin, ymin, xmax, ymax = detections[0, 0, i, 3], detections[0, 0, i, 4], \
                                        detections[0, 0, i, 5], detections[0, 0, i, 6]
                                
                d = Detection2D()
                d.bbox.size_x = xmax - xmin
                d.bbox.size_y = ymax - ymin
                d.bbox.center.x = xmin + d.bbox.size_x/2
                d.bbox.center.y = ymin + d.bbox.size_y/2
                o = ObjectHypothesisWithPose()
                o.score = score
                o.id = 2
                d.results.append(o)
                rospy.logdebug_once("Face Detected!")
                message.detections.append(d)
                            
        self._publisher.publish(message)

if __name__ == '__main__':
    face_detector_node = FaceDetectorNode()
    face_detector_node()