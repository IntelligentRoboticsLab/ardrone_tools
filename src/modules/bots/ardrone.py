'''This file is implementing the AR.Drone and handles controls. Every drone
has a computer vision object, which contains methods to follow lines and
dynamic objects.
'''
import uuid
import subprocess

import rospy
import std_srvs.srv
import std_msgs.msg
from geometry_msgs.msg import Twist

from ..vision.vision import ComputerVision
from bot import Bot
from sensors import Camera, IMU, NavData

class ARDrone(Bot):
    '''Class implementing the AR.Drone'''

    def __init__(self, name='/drone0', ip='192.168.1.1', use_simulator=False,
        recording=False):
        '''Constructor of the AR.Drone'''
        Bot.__init__(self)
        self.NAME = name
        self.IP = ip
        self.UID = uuid.uuid4()
        
        if not use_simulator and not recording:
            self.driver = subprocess.Popen(['rosrun', 'ardrone_autonomy',
                'ardrone_driver', '__ns:=' + name, '-ip', ip])
        else:
            self.driver = None

        # Sensors
        cam_front = Camera(name + '/ardrone/front/image_raw')
        cam_bottom = Camera(name + '/ardrone/bottom/image_raw')
        self.CAMERAS = [cam_front, cam_bottom]
        self.CAMERA_ID = 0
        self.NAVDATA = NavData(name + '/ardrone/navdata')
        self.IMU = IMU(name + '/ardrone/imu')

        # Controlling the AR.Drone
        self.CONTROLLER = rospy.Publisher(name + '/cmd_vel', Twist)
        self.CONTROLS = Twist()

        # Variables specificly for AR.Drone
        self.pub_land = rospy.Publisher(name + '/ardrone/land',
                                        std_msgs.msg.Empty)
        self.pub_take_off = rospy.Publisher(name + '/ardrone/takeoff',
                                            std_msgs.msg.Empty)
        self.pub_reset = rospy.Publisher(name + '/ardrone/reset',
                                         std_msgs.msg.Empty)

        # Variables useful ui information.
        self.states = {
            0: 'Unknown',
            1: 'Init',
            2: 'Landed',
            3: 'Flying',
            4: 'Hovering',
            5: 'Test',
            6: 'Taking off',
            7: 'Flying',
            8: 'Landing',
            9: 'Looping'
        }
                       
        self.cv = ComputerVision(self)
        self.autonomy = False
        
    def shutdown(self):
        '''Shuts down the driver of the AR.Drone'''
        print "%s Terminated!" % (self.NAME)
        if self.driver:
            self.switch_off_autonomy()
            self.land()
            self.driver.send_signal(subprocess.signal.SIGINT)
            
        self.cv.shutdown()

    def toggle_airborne(self):
        '''Changes the state of the AR.Drone'''
        navdata = self.NAVDATA.get()
        if navdata:
            state = navdata.state
            if state != 2:
                self.land()
            else:
                self.take_off()
        else:
            self.land()

    def land(self):
        '''Sends a landing message to the AR.Drone'''
        print "Landing"
        self.pub_land.publish(std_msgs.msg.Empty())

    def take_off(self):
        '''Sends a take off message to the AR.Drone '''
        print "Taking off"
        self.pub_take_off.publish(std_msgs.msg.Empty())

    def reset(self):
        '''Sends a reset message to the AR.Drone'''
        self.pub_reset.publish(std_msgs.msg.Empty())

    def toggle_cam(self):
        '''Switches the camera of the AR.Drone'''
        #rospy.wait_for_service(self.NAME + '/ardrone/togglecam')
        try:
            toggle = rospy.ServiceProxy(self.NAME + '/ardrone/togglecam',
                                        std_srvs.srv.Empty)
            toggle()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        if self.CAMERA_ID < len(self.CAMERAS) - 1:
            self.CAMERA_ID += 1
        else:
            self.CAMERA_ID = 0

    def toggle_autonomy(self):
        '''Switches autonomy'''
        self.autonomy = not self.autonomy
        
    def switch_on_autonomy(self):
        '''Switches autonomy on'''
        self.autonomy = True
        
    def switch_off_autonomy(self):
        '''Switches autonomy off'''
        self.autonomy = False

    def flat_trim(self):
        '''Flat trims the sensors'''
        #rospy.wait_for_service(self.NAME + '/ardrone/flattrim')
        try:
            toggle = rospy.ServiceProxy(self.NAME + '/ardrone/flattrim',
                                        std_srvs.srv.Empty)
            toggle()
            if self.CAMERA_ID < len(self.CAMERAS) - 1:
                self.CAMERA_ID += 1
            else:
                self.CAMERA_ID = 0
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def move(self):
        '''Sends controls to the AR.Drone'''
        if self.autonomy:
            controls = self.cv.get_controls()
            if controls:
                self.set_control(**controls)
            else:
                self.CONTROLS = Twist()
        self.CONTROLLER.publish(self.CONTROLS)

    def set_control(self, **kwargs):
        '''Setting the controls'''
        for key, value in kwargs.items():
            if key == 'linearx':
                self.CONTROLS.linear.x = value
            elif key == 'lineary':
                self.CONTROLS.linear.y = value
            elif key == 'linearz':
                self.CONTROLS.linear.z = value
            elif key == 'angularz':
                self.CONTROLS.angular.z = value
        
    def get_imu(self):
        '''Returns the imu data'''
        return self.IMU.get()

    def get_navdata(self):
        '''Returns the navigation data'''
        return self.NAVDATA.get()

    def get_state(self):
        '''Returns a string with the current state'''
        navdata = self.NAVDATA.get()
        if navdata:
            return self.states[navdata.state]
        else:
            return 'Unknown'

    def get_autonomy(self):
        '''Returns the autonomy status'''
        return self.autonomy
        
    def get_frame(self):
        '''Returns the frame of the current camera as ROS imgmsg'''
        return self.CAMERAS[self.CAMERA_ID].get()