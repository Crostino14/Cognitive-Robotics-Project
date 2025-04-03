import rospy

class ServiceHandler:
    """
    A utility class for managing and interacting with ROS (Robot Operating System) services.
    This class provides methods to initialize, call, and close both persistent and non-persistent ROS service connections.

    Attributes:
    - _services (dict): Dictionary storing non-persistent service connections.
    - _persistent_services (dict): Dictionary storing persistent service connections.

    Methods:
    - _init_service(service_name: str, service_class: any, persistent: bool): Initializes a ROS service connection.
    - __call__(service_name: str, *args): Calls a ROS service with the given arguments.
    - _close_services(): Closes persistent service connections.
    - __str__(): Returns a string representation of all managed services.
    """

    def __init__(self):
        """
        Initialize the ServiceHandler class.

        Attributes:
        - _services (dict): Dictionary to store non-persistent service connections.
        - _persistent_services (dict): Dictionary to store persistent service connections.
        """
        self._services = {}
        self._persistent_services = {}

    def _init_service(self, service_name: str, service_class: any, persistent: bool = False):
        """
        Initialize a service connection.

        Args:
        - service_name (str): The name of the ROS service to connect to.
        - service_class (any): The service class defining the type of the ROS service.
        - persistent (bool): Whether to use a persistent connection (default: False).

        Note:
        Persistent connections improve performance for repeated requests but require additional reconnection logic
        in case the connection fails. Use them cautiously.
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
        Call a service with the provided arguments.

        Args:
        - service_name (str): The name of the ROS service.
        - *args: Variable number of arguments to pass to the service.

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
        Close all persistent service connections managed by the handler.

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
        Return a string representation of all services managed by the handler.

        Returns:
        - str: A list of all managed service names.
        """
        return '\n'.join([f"Service {service}" for service in self._services.keys()])