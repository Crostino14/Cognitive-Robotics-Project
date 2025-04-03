#!/usr/bin/python3
import rospy

from shopping_mall_assistant.srv import ASRService, VoiceDetectionService

class AudioTestNode():
    """
    A ROS (Robot Operating System) node for testing audio input and transcription services.

    This node integrates two key services:
    - Voice detection: Captures audio data using the `voice_detection_service`.
    - Speech-to-text (ASR): Converts captured audio data into text using the `asr_service`.

    Attributes:
    - _services (ServiceHandler): Manages persistent ROS service connections.

    Methods:
    - __call__(): Main loop to capture and process audio continuously.
    - __handle_voice_detection(): Calls the `voice_detection_service` to capture audio.
    - __handle_s2t(audio): Sends audio data to the `asr_service` for transcription.
    - __handle_shutdown(): Closes all persistent service connections during shutdown.
    """

    def __init__(self):      
        """
        Initializes the AudioTestNode class, setting up ROS services and shutdown handlers.

        Services:
        - asr_service: Speech-to-text service.
        - voice_detection_service: Voice detection service.
        """
        # Init Ros Node
        rospy.init_node('audio_test')
        rospy.on_shutdown(self.__handle_shutdown)
        # Init Services
        self._services = ServiceHandler()
        self._services._init_service('asr_service', ASRService, persistent=True)
        self._services._init_service('voice_detection_service', VoiceDetectionService, persistent=True)
        
    def __call__(self):
        """
        Main execution loop for the AudioTestNode.

        Continuously captures audio data and processes it through the ASR service.
        Logs warnings if no audio is detected or no text is generated.
        """
        while not rospy.is_shutdown():            
            audio = self.__handle_voice_detection()
            if audio.data == (0, 0):
                rospy.logwarn("No audio detected")
                continue
            text = self.__handle_s2t(audio)
            if text:
                rospy.logdebug(f"\n\n\nGenerated text: {text}\n\n\n")
            else:
                rospy.logdebug("No text generated")
               
    def __handle_voice_detection(self):
        """
        Calls the `voice_detection_service` to capture audio data.

        Returns:
        - output (std_msgs.msg.Int16MultiArray): The captured audio data.
        """
        audio_data = self._services('voice_detection_service')
        return audio_data.output
       
    def __handle_s2t(self, audio):
        """
        Sends captured audio data to the `asr_service` for transcription.

        Args:
        - audio (std_msgs.msg.Int16MultiArray): Captured audio data.

        Returns:
        - str: The transcribed text if successful; None otherwise.
        """
        s2t = self._services('asr_service', audio)
        return s2t.output.data if s2t is not None else None
    
    def __handle_shutdown(self):
        """
        Closes all persistent ROS service connections during node shutdown.
        """
        self._services._close_services()
    
class ServiceHandler:
    """
    A utility class for managing and interacting with ROS (Robot Operating System) services.

    This class handles both persistent and non-persistent service connections, ensuring seamless
    communication with ROS services.

    Attributes:
    - _services (dict): Dictionary to store service connections.
    - _persistent_services (dict): Dictionary to store persistent service connections.

    Methods:
    - _init_service(service_name: str, service_class: any, persistent: bool): Initializes a ROS service connection.
    - __call__(service_name: str, *args): Calls a ROS service with the given arguments.
    - _close_services(): Closes all persistent service connections.
    - __str__(): Returns a string representation of all managed services.
    """

    def __init__(self):
        """
        Initializes the ServiceHandler class, setting up dictionaries for service management.
        """
        self._services = {}
        self._persistent_services = {}

    def _init_service(self, service_name: str, service_class: any, persistent: bool = False):
        """
        Initializes a ROS service connection.

        Args:
        - service_name (str): The name of the ROS service to connect to.
        - service_class (any): The service class defining the service type.
        - persistent (bool): Whether to use a persistent connection (default: False).

        Notes:
        Persistent connections improve performance for repeated requests but require additional
        reconnection logic if the connection fails.
        """
        try:
            if persistent:
                # Initialize persistent service connection
                service_proxy = rospy.ServiceProxy(service_name, service_class, persistent=True)
                self._persistent_services[service_name] = service_proxy
            else:
                # Initialize non-persistent service connection
                rospy.wait_for_service(service_name)
                service_proxy = rospy.ServiceProxy(service_name, service_class, persistent=False)

            # Store the service connection in the dictionary
            self._services[service_name] = service_proxy

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to initialize service '{service_name}': {e}")

    def __call__(self, service_name: str, *args) -> any:
        """
        Calls a ROS service with the provided arguments.

        Args:
        - service_name (str): The name of the ROS service to call.
        - *args: Variable arguments to pass to the service.

        Returns:
        - Any: The response from the service call.

        Raises:
        - rospy.ServiceException: If the service call fails.
        """
        try:
            if args:
                response = self._services[service_name](*args)
            else:
                response = self._services[service_name]()
            return response
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call service '{service_name}': {e}")
            return None
            
    def _close_services(self):
        """
        Closes all persistent ROS service connections managed by the handler.

        Note:
        Non-persistent connections do not require manual closing as they are closed automatically
        after each service call.
        """
        try:
            for service_name in self._persistent_services.keys():
                self._services[service_name].close()
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to close persistent services: {e}")

    def __str__(self) -> str:
        """
        Returns a string representation of all managed services.

        Returns:
        - str: A list of all managed service names.
        """
        return '\n'.join([f"Service {service}" for service in self._services.keys()])    

if __name__ == '__main__':
   audio_test_node = AudioTestNode()
   audio_test_node()