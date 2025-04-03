#!/usr/bin/python3
import rospy

from optparse import OptionParser
from std_msgs.msg import String
from session import *

from shopping_mall_assistant.srv import WakeUp, WakeUpResponse, Rest, RestResponse

class WakeUpRestNode:
    '''
    A ROS (Robot Operating System) node specifically designed for controlling the wake-up and rest states of a robot, 
    particularly for the Pepper robot. This node enables the robot to transition between active and rest states 
    and manage its posture accordingly.

    Attributes:
    - _session (Session): A session object to communicate with the robot.
    - _motion_proxy (ALMotion): A service for controlling the robot's motion.
    - _posture_proxy (ALRobotPosture): A service for managing the robot's posture.
    - _ba_service (ALBasicAwareness): The basic awareness service of the robot.

    Methods:
    - __init__(self, ip: str, port: int): Initializes the WakeUpRestNode with the robot's IP and port.
    - __call__(self): Starts the ROS node and initializes the wake-up process for the robot.
    - __stand(self, *args: any): Sets the robot to a standard initial posture.
    - __rest(self, *args: any): Transitions the robot to a rest state and disables motion.
    - __wakeup(self, *args: any): Activates the robot from its rest state and sets it to an initial posture.
    '''
    
    def __init__(self, ip : str, port : int) -> None:
        '''
        Initializes the WakeUpRestNode. Establishes a session with the Pepper robot and initializes services 
        for motion control, posture management, and basic awareness.

        Args:
        - ip (str): The IP address of the robot.
        - port (int): The port number for the connection.
        '''
        self._session = Session(ip, port)
        self._motion_proxy = self._session.get_service("ALMotion")
        self._posture_proxy = self._session.get_service("ALRobotPosture")
        self._ba_service = self._session.get_service("ALBasicAwareness")
        
    
    def __call__(self) -> None:
        '''
        Starts the ROS node and initializes the wake-up service. This method sets up the services for waking up 
        and resting the robot and keeps the node active until shutdown.
        '''
        rospy.init_node("wakeup_rest_node")
        self.__wakeup()    
        rospy.Service("wakeup_service", WakeUp, self.__wakeup)
        rospy.Service("rest_service", Rest, self.__rest)
        rospy.spin()
        rospy.on_shutdown(self.__rest)
                
    def __stand(self, *args: any) -> None:
        '''
        Sets the robot to its initial 'StandInit' posture. This method is used to ensure the robot is in a 
        standard posture after waking up.

        Args:
        - args: Additional arguments (not used in this implementation).
        '''
        self._posture_proxy.goToPosture("StandInit", 0.5)
    
    def __rest(self, *args : any) -> RestResponse:
        '''
        Handles the robot's transition to a resting state. Disables basic awareness and motion, ensuring the robot 
        is in a safe, inactive state. In case of an exception, it resets the motion service and retries.

        Args:
        - args: Additional arguments (not used in this implementation).

        Returns:
        - RestResponse: A response message indicating the robot is in a rest state.
        '''
        try:
            self._ba_service.setEnabled(False)
            #if self._motion_proxy.robotIsWakeUp():
            #    self._motion_proxy.rest()
        except:
            self._motion_proxy = self._session.get_service("ALMotion")
            self._ba_service.setEnabled(False)
            #if self._motion_proxy.robotIsWakeUp():
            #    self._motion_proxy.rest()
        return RestResponse(String("ACK"))
    
    def __wakeup(self, *args: any) -> WakeUpResponse:
        '''
        Wakes the robot from its resting state. Enables motion and basic awareness services and sets the robot to 
        its initial posture. In case of an exception, it resets the necessary services and retries the wake-up process.

        Args:
        - args: Additional arguments (not used in this implementation).

        Returns:
        - WakeUpResponse: A response message indicating the robot has been woken up.
        '''
        try:
            if not self._motion_proxy.robotIsWakeUp():
                self._motion_proxy.wakeUp()
            self._ba_service.setEngagementMode("FullyEngaged")
            self._ba_service.setEnabled(True)
            self._ba_service.setTrackingMode("MoveContextually")
            self.__stand()
        except:
            self._motion_proxy = self._session.get_service("ALMotion")
            self._posture_proxy = self._session.get_service("ALRobotPosture")
            if not self._motion_proxy.robotIsWakeUp():
                self._motion_proxy.wakeUp()
            self._ba_service.setEngagementMode("FullyEngaged")
            self._ba_service.setEnabled(True)
            self.__stand()         

        return WakeUpResponse(String("ACK"))

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    wakeup_rest_node = WakeUpRestNode(options.ip, int(options.port))
    wakeup_rest_node()
