#!/usr/bin/python3
import qi
import sys

class Session:
    '''
    This class creates a qi session using the ip and port parameters provided as input to costructor
    '''

    def __init__(self, ip, port):
        '''
        The costructor creates a qi session object. It then uses the ip and port parameters to connect the object to Pepper OS
        '''
        self.ip = ip
        self.port = port
        self._session = qi.Session()
        self._connect()

    def _connect(self):
        '''
        This method uses the session object to connect to Pepper OS. If an exception occurs, the application is killed.
        '''
        try:
            self._session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("Can't connect to Naoqi at ip \"" + self.ip + "\" on port " + str(self.port) + ".\n "
                                                                                                 "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

    def reconnect(self):
        '''
        This method reconnect the session object to Pepper OS
        @return: Returns the session object
        '''
        self._connect()
        return self._session
    
    @property
    def session(self):
        '''
        Getter for the session object
        '''
        return self._session
    
    def get_service(self, service_name: str):
        '''
        This method returns the NaoQi service given as parameter.
        @param service_name: The name of the NaoQi service in form of string
        @return: Returns the service object
        '''
        return self._session.service(service_name)