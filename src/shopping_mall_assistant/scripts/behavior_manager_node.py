#!/usr/bin/python3
import rospy

from optparse import OptionParser
from service_handler import ServiceHandler
from std_msgs.msg import String
from threading import Lock, Thread
from vision_msgs.msg import Detection2DArray

from shopping_mall_assistant.srv import ASRService, Dialogue, VoiceDetectionService, AnimatedSpeechService

class BehaviorManagerNode():
    '''
    A ROS (Robot Operating System) node for managing the overall behavior of a robot. 
    It handles interactions with humans through face detection, voice input, dialogue processing, 
    and synchronized speech with animations.

    This node integrates multiple services such as speech-to-text (ASR), dialogue, voice detection, 
    and animated speech management, ensuring that the robot can engage interactively with its users.

    Attributes:
    - _pepper_on (bool): Indicates whether the robot (Pepper) is active.
    - _services (ServiceHandler): Manages ROS services for various functionalities.
    - _face_lock (Lock): Ensures thread-safe access to face detection data.
    - _face_presence (bool): Tracks whether a face is currently detected.
    - _last_face_presence (bool): Stores the previous face detection state.
    - _time_face_presence (float): Timestamp of the last detected face.

    Methods:
    - __init__(self, pepper_on): Initializes the node and sets up subscribers and services.
    - __call__(self): Main loop handling interaction logic based on inputs and service responses.
    - __check_face_presence(self): Verifies the presence of a face in the robot's view.
    - __handle_t2s(self, text): Sends text to the animated speech service for synchronized speech and gestures.
    - __handle_chatbot(self, text): Sends input text to the chatbot service and retrieves the response.
    - __handle_face_detection(self, msg): Callback to process face detection updates.
    - __handle_voice_detection(self): Calls the voice detection service to retrieve audio data.
    - __handle_s2t(self, audio): Converts audio data to text using the ASR service.
    - __handle_shutdown(self): Cleans up services during node shutdown.
    '''

    def __init__(self, pepper_on: bool):
        '''
        Initializes the BehaviorManagerNode, setting up required ROS services and subscribers.

        Args:
        - pepper_on (bool): Indicates if the robot is active and additional services should be initialized.
        '''
        self._pepper_on = pepper_on
        self._services = ServiceHandler()
        self._face_lock = Lock()
        self._face_presence = False
        self._last_face_presence = False
        self._time_face_presence = 0

        # Initialize ROS Node
        rospy.init_node('behavior_manager')
        rospy.Subscriber("/face_detection", Detection2DArray, self.__handle_face_detection)
        rospy.on_shutdown(self.__handle_shutdown)

        # Initialize Services
        self._services._init_service('asr_service', ASRService, persistent=True)
        self._services._init_service('dialogue_server', Dialogue, persistent=True)
        self._services._init_service('voice_detection_service', VoiceDetectionService, persistent=True)
        if self._pepper_on:
            self._services._init_service('animated_speech_service', AnimatedSpeechService, persistent=True)

    def __call__(self):
        '''
        Main execution loop for the node. This method checks for face presence and handles interactions 
        based on detected inputs like voice or dialogue, coordinating appropriate responses with speech and animations.
        '''
        rospy.loginfo("Behavior Manager Ready!")
        rate = rospy.Rate(10)  # 10 Hz loop rate

        while not rospy.is_shutdown():
            rate.sleep()

            if not self.__check_face_presence():
                continue

            audio = self.__handle_voice_detection()
            if not audio or audio.data == (0, 0):
                continue

            if not self.__check_face_presence():
                continue

            text = self.__handle_s2t(audio)
            if not self.__check_face_presence() or text == "":
                continue

            answer = self.__handle_chatbot(text)
            rospy.loginfo("Answer: \n\n%s\n\n", answer)

            if self._pepper_on and answer != "":
                self.__handle_t2s(answer)

    # Private Methods
    def __check_face_presence(self):
        '''
        Checks if a face is currently present. If a face has not been detected for a set duration, 
        the chatbot interaction is reset.

        Returns:
        - bool: True if a face is present, False otherwise.
        '''
        with self._face_lock:
            current_time = rospy.get_time()
            try:
                if not self._face_presence and (current_time - self._time_face_presence) > 20:
                    self.__handle_chatbot("/restart")
                    rospy.logwarn(f"Face not detected for more than {int(current_time - self._time_face_presence)} seconds. Resetting interaction.")
                    return False
            except AttributeError:
                return False
            return True

    def __handle_t2s(self, text):
        '''
        Sends text to the animated speech service for synchronized speech and gestures.

        Args:
        - text (str): The text to be spoken by the robot.
        '''
        self._services('animated_speech_service', String(text))

    def __handle_chatbot(self, text="Hello there!"):
        '''
        Sends input text to the chatbot service and retrieves the chatbot's response.

        Args:
        - text (str, optional): Input text for the chatbot. Defaults to "Hello there!".

        Returns:
        - str: The chatbot's response.
        '''
        response = self._services('dialogue_server', text.lower())
        return response.answer

    def __handle_face_detection(self, msg):
        '''
        Updates face presence information based on detection data.

        Args:
        - msg (Detection2DArray): Message containing face detection data.
        '''
        self._last_face_presence = self._face_presence
        self._face_presence = len(msg.detections) > 0
        if self._face_presence:
            self._time_face_presence = rospy.get_time()

    def __handle_voice_detection(self):
        '''
        Detects voice input by calling the voice detection service.

        Returns:
        - The output of the voice detection service (typically audio data).
        '''
        try:
            audio_data = self._services('voice_detection_service')
            return audio_data.output
        except AttributeError:
            return False

    def __handle_s2t(self, audio):
        '''
        Converts audio data to text using the ASR (Automatic Speech Recognition) service.

        Args:
        - audio: The audio data to convert to text.

        Returns:
        - str: The text converted from the audio input.
        '''
        s2t = self._services('asr_service', audio)
        return s2t.output.data if s2t else None

    def __handle_shutdown(self):
        '''
        Cleans up services during node shutdown.
        '''
        self._services._close_services()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--pepper_on", dest="pepper_on", default="True")
    (options, args) = parser.parse_args()

    pepper_on = (options.pepper_on.lower() == "true")
    behavior_manager_node = BehaviorManagerNode(pepper_on=pepper_on)
    behavior_manager_node()