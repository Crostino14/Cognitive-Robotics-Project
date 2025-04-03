#!/usr/bin/python3
import numpy
import rospy

from optparse import OptionParser
from std_msgs.msg import String
from session import *

from shopping_mall_assistant.srv import AnimatedSpeechService, AnimatedSpeechServiceResponse

class AnimatedSpeechNode():
    '''
    A ROS (Robot Operating System) node designed for managing animated speech in a robot. 
    This node communicates with the robot's ALAnimatedSpeech service, allowing the robot 
    to execute animations synchronized with speech based on the provided commands.

    Attributes:
    - _session (Session): A session object for communication with the robot.
    - _animated_speech (ALAnimatedSpeech): The animated speech service used to execute 
      synchronized speech and gestures on the robot.

    Methods:
    - __init__(self, ip, port): Initializes the node, setting up a session and the ALAnimatedSpeech service.
    - __call__(self): Starts the ROS node and registers the animated speech service.
    - __animateSpeech(self, msg: AnimatedSpeechService) -> AnimatedSpeechServiceResponse:
      Handles requests to execute speech with synchronized animations.
    '''
    
    def __init__(self, ip, port):
        '''
        Initializes the AnimatedSpeechNode, creating a connection to the robot using the provided 
        IP address and port. This method sets up the session and initializes the ALAnimatedSpeech service.

        Args:
        - ip (str): The IP address of the robot.
        - port (int): The port number for establishing the connection.
        '''
        self._session = Session(ip, port)
        self._animated_speech = self._session.get_service("ALAnimatedSpeech")
        self._t2s = self._session.get_service("ALTextToSpeech")
        self._t2s.setLanguage("English")
        self._t2s.setVolume(1.0)

    def __call__(self):
        '''
        Starts the ROS node and registers the Animated Speech Service. This method creates the service 
        'animated_speech_service' to handle speech and animation requests. The node remains active 
        until it is interrupted.
        '''
        rospy.init_node("animated_speech_node")
        rospy.Service('animated_speech_service', AnimatedSpeechService, self.__animateSpeech)
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    # Private Methods
    def __animateSpeech(self, msg: AnimatedSpeechService) -> AnimatedSpeechServiceResponse:
        '''
        Handles animated speech requests by interpreting the input text and executing the corresponding speech 
        with synchronized animations. The method supports predefined animations and contextual gestures 
        using the ALAnimatedSpeech service.

        Args:
        - msg (AnimatedSpeechService): The input message containing the text with optional animation tags.

        Returns:
        - AnimatedSpeechServiceResponse: A response message indicating the successful execution of the speech.
        '''
        speech_text = msg.input.data  # Text with optional animation tags
        try:
            # Configure the ALAnimatedSpeech options to enable contextual gestures
            config = {"bodyLanguageMode": "contextual"}  
            self._animated_speech.say(speech_text, config)
        except Exception as e:
            rospy.logerr(f"Error with ALAnimatedSpeech: {e}")
        return AnimatedSpeechServiceResponse(String("ACK"))

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    animation_node = AnimatedSpeechNode(options.ip, int(options.port))
    animation_node()
