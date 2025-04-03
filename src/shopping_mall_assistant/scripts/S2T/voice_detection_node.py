#!/usr/bin/python3
from optparse import OptionParser

import numpy as np
import rospy
import speech_recognition as sr

from std_msgs.msg import Int16MultiArray
from shopping_mall_assistant.srv import VoiceDetectionService, VoiceDetectionServiceResponse

class VoiceDetection:
    """
    A ROS (Robot Operating System) node for detecting voice input from a microphone.
    This node captures audio data, processes it, and returns it in a format compatible with ROS services.

    Attributes:
    - _recognizer (sr.Recognizer): The recognizer object for managing audio input.
    - _microphone (sr.Microphone): The microphone object for capturing audio.
    - _device_index (int): The index of the microphone device to be used.

    Methods:
    - __init__(self, device_index): Initializes the VoiceDetection class with the specified microphone.
    - __call__(self): Starts the ROS node and sets up the voice detection service.
    - __handle_audio_recording(self, msg): Handles the service request for audio recording.
    """

    def __init__(self, device_index):
        """
        Initializes the VoiceDetection class.

        Args:
        - device_index (int): Index of the microphone device to use.
        """
        # Initialize a Recognizer
        self._recognizer = sr.Recognizer()
        self._recognizer.dynamic_energy_threshold = False
        self._recognizer.energy_threshold = 80

        self._device_index = device_index

        # List available microphones for debugging
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            print("Microphone with name '{}' found for Microphone(device_index={})".format(name, index))

        # Initialize a Microphone
        self._microphone = sr.Microphone(device_index=device_index, sample_rate=16000, chunk_size=1024)

    def __call__(self):
        """
        Starts the ROS node and sets up the 'voice_detection_service'.
        The node listens for service requests to capture audio data.
        """
        rospy.init_node('voice_detection', anonymous=True)
        rospy.Service('voice_detection_service', VoiceDetectionService, self.__handle_audio_recording, buff_size=1)
        rospy.loginfo("Voice Detection Node is listening...")
        rospy.loginfo(f"Using microphone with device index: {self._device_index}")

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Voice Detection Node shutting down.")

    def __handle_audio_recording(self, msg):
        """
        Handles the audio recording service request.

        Args:
        - msg: The service request (not used in this case).

        Returns:
        - VoiceDetectionServiceResponse: Contains the captured audio data as Int16MultiArray.
        """
        try:
            rospy.loginfo("Recording audio...")
            with self._microphone as source:
                self._recognizer.adjust_for_ambient_noise(source, duration=3)
                audio = self._recognizer.listen(source, timeout=7)
                data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
                data_to_send = Int16MultiArray(data=data.tolist())
                rospy.loginfo("Audio captured successfully.")
                return VoiceDetectionServiceResponse(data_to_send)
        except sr.WaitTimeoutError:
            rospy.logwarn("Listening timed out. No speech detected within the timeout period.")
            data_to_send = Int16MultiArray(data=[0, 0])
            return VoiceDetectionServiceResponse(data_to_send)
        except sr.UnknownValueError:
            rospy.logwarn("No speech detected or unclear audio.")
            data_to_send = Int16MultiArray(data=[0, 0])
            return VoiceDetectionServiceResponse(data_to_send)
        except sr.RequestError as e:
            rospy.logerr(f"Error during audio recording: {e}")
            data_to_send = Int16MultiArray(data=[0, 0])
            return VoiceDetectionServiceResponse(data_to_send)

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--mic_index", dest="mic_index", default="11")
    (options, args) = parser.parse_args()

    # Parse the microphone index from command-line arguments
    device_index = int(options.mic_index) if options.mic_index != "None" else None

    # Initialize and run the VoiceDetection node
    voice_detection = VoiceDetection(device_index)
    voice_detection()