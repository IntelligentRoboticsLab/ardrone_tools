'''This file contains sensor interfaces.
'''
from threading import Lock

import rospy
from sensor_msgs.msg import Image, Imu
from ardrone_autonomy.msg import Navdata

class Sensor(object):
    '''Class implementing a Sensor'''

    def __init__(self):
        '''Initializes the sensor given a topic'''
        self.LOCK   = Lock()
        self.VALUE  = None
        self.SENSOR = None 

    def callback(self, value):
        '''The callback function'''
        self.LOCK.acquire()
        self.VALUE = value
        self.LOCK.release()

    def get(self):
        '''Get function returning the value of the sensor'''
        self.LOCK.acquire()
        value = self.VALUE
        self.LOCK.release()

        if value:
            return value
        else:
            return False


class Camera(Sensor):
    '''Class implementing a camera'''

    def __init__(self, name = None):
        '''Initializes the camera given a topic'''
        assert name != None
        super(Camera, self).__init__()
        self.SENSOR = rospy.Subscriber(name, Image,
                                       super(Camera, self).callback)

class IMU(Sensor):
    '''Class implementing a IMU'''

    def __init__(self, name = None):
        '''Initializes the IMU given a topic'''
        assert name != None
        super(IMU, self).__init__()
        self.SENSOR = rospy.Subscriber(name, Imu, super(IMU, self).callback)


class NavData(Sensor):
    '''Class implementing NavData'''

    def __init__(self, name = None):
        '''Initializes the NavData given a topic'''
        assert name != None
        super(NavData, self).__init__()
        self.SENSOR = rospy.Subscriber(name, Navdata,
                                       super(NavData, self).callback)