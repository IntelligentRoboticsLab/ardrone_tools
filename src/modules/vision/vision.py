'''This file implements all the computer vision algorithms used by the drones
'''
import time
import math
import subprocess
import threading

import cv_bridge
import cv
import rospy
import numpy as np
from scipy.ndimage import interpolation

class ComputerVision():
    '''Implements all the Computer Vision methods of the drone'''

    def __init__(self, bot=None):
        '''Constructor of a computer vision object'''
        assert bot != None
        self.bot = bot
        self.bridge = cv_bridge.CvBridge()
        self.hsv_values = [(0, 0, 0, 0), (180, 255, 255, 0)]
        self.rectangle = (0, 0, 0, 0)
        self.opentld = OpenTLD(self)
        self.follow_line = FollowLine(self)
        self.algorithm = None

    def shutdown(self):
        '''Shuts down all the algorithms'''
        self.opentld.shutdown()
        self.follow_line.shutdown()
        
    def get_frame(self, encoding='bgr8', resolution=None, hsv_filter=False,
                  draw_rectangle=False, draw_lines=False):
        '''Returns frame in OpenCV format'''
        frame = self.bot.get_frame()
        if not frame:
            return False
        if hsv_filter:
            frame = self.imgmsg_to_cv(frame, 'bgr8')
            frame = self.hsv_filter(frame, encoding)
        else:
            frame = self.imgmsg_to_cv(frame, encoding)
        if draw_rectangle:
            frame = self.draw_rectangle(frame)
        if draw_lines:
            frame = self.draw_lines(frame)
        if resolution:
            frame = self.resize(frame, resolution)
        return frame
    
    def opentld_setup(self):
        '''Sets up the opentld driver'''
        self.opentld.setup()
        
    def opentld_start(self, frame):
        '''Starts opentld'''
        new_frame = self.cv_to_imgmsg(frame, "rgb8")
        self.opentld.send_target(new_frame, self.rectangle)
        self.bot.switch_on_autonomy()
        self.algorithm = self.opentld_controls
        
    def opentld_stop(self):
        '''Stops opentld'''
        self.bot.switch_off_autonomy()
        self.algorithm = None
    
    def opentld_controls(self):
        '''Return the controls generated by opentld'''
        return self.opentld.get_controls()
        
    def get_target_opentld(self):
        '''Returns the target found by opentld'''
        return self.opentld.get_target()
        
    def opentld_is_active(self):
        return self.opentld.is_active()
        
    def get_controls(self):
        '''Returns the controls generated by the active algorithm'''
        if not self.algorithm:
            return False
        return self.algorithm()
        
    def follow_line_start(self):
        '''Starts the follow line module'''
        self.follow_line = FollowLine(self)
        self.bot.switch_on_autonomy()
        self.algorithm = self.follow_line_controls
        self.follow_line.start()
        
    def follow_line_stop(self):
        '''Stops the follow line module'''
        self.bot.switch_off_autonomy()
        self.follow_line.shutdown()
        self.algorithm = None
        
    def follow_line_controls(self):
        '''Returns the follow line controls'''
        return self.follow_line.get_controls()

    def imgmsg_to_cv(self, frame, encoding):
        '''Transforms a ros frame into a opencv frame'''
        return self.bridge.imgmsg_to_cv(frame, encoding)

    def cv_to_imgmsg(self, frame, encoding):
        '''Transforms a opencv frame into a ros frame'''
        return self.bridge.cv_to_imgmsg(frame, encoding)
        
    def hsv_filter(self, frame, encoding):
        '''Filters frame in a certain HSV range'''
        hsv_frame = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 3)
        filter_frame = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)        
        cv.CvtColor(frame, hsv_frame, cv.CV_BGR2HSV)            
        cv.InRangeS(hsv_frame, self.hsv_values[0], self.hsv_values[1],
                    filter_frame)
        
        if encoding == 'rgb8':
            rgb_frame = cv.CreateImage(cv.GetSize(filter_frame),
                                       cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(filter_frame, rgb_frame, cv.CV_GRAY2RGB)
            return rgb_frame
        elif encoding == 'bgr8':
            bgr_frame = cv.CreateImage(cv.GetSize(filter_frame),
                                       cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(filter_frame, rgb_frame, cv.CV_GRAY2BGR)
            return bgr_frame
        return filter_frame

    def resize(self, frame, resolution):
        '''Resizes a frame to a resolution'''
        thumbnail = cv.CreateMat(resolution[0], resolution[1], frame.type)
        cv.Resize(frame, thumbnail)
        return thumbnail

    def set_hsv_filter(self, hsv_min, hsv_max):
        '''Sets the HSV filter'''
        self.hsv_values = [hsv_min, hsv_max]
        
    def set_rectangle(self, rectangle):
        '''Sets the rectangle'''
        self.rectangle = rectangle 
        
    def draw_rectangle(self, frame):
        '''Draws a rectangle on a frame'''
        cv.Rectangle(frame, (self.rectangle[0], self.rectangle[1]),
                     (self.rectangle[2], self.rectangle[3]), (0, 0, 255), 2)        
        return frame
        
    def draw_lines(self, frame):
        '''Draws lines'''
        if self.follow_line.is_active():
            cv.Line(frame, self.follow_line.get_center(),
                    self.follow_line.get_max(), cv.CV_RGB(0, 100, 100), 3, 8)
            cv.Line(frame, self.follow_line.get_center(),
                    self.follow_line.get_mean(), cv.CV_RGB(0, 255, 0), 3, 8)
            return frame
        return frame

        
class OpenTLD():
    '''This class implements the functionality of OpenTLD'''

    def __init__(self, vision):
        '''Construct of OpenTLD'''
        from tld_msgs.msg import BoundingBox, Target
        self.vision = vision
        self.pub_target = rospy.Publisher(
            self.vision.bot.NAME + '/bounding_box', Target)
        self.sub_tracked_object = rospy.Subscriber(
            self.vision.bot.NAME + '/tld_tracked_object', BoundingBox,
            self.callback)
        self.target = Target()
        self.tracked_target = None
        self.driver = None
        self.controls = dict()
        
        self.time_last_detection = time.time()

    def shutdown(self):
        '''Shuts down the OpenTLD driver'''
        if self.driver:
            self.driver.send_signal(subprocess.signal.SIGINT)
            self.driver = None
            
    def is_active(self):
        '''Checks if the driver is setup'''
        if self.driver:
            return True
        return False
        
    def setup(self):
        '''Sets up the driver of OpenTLD'''
        if not self.driver:
            self.driver = subprocess.Popen(['rosrun', 'tld_tracker',
                'ros_tld_tracker_node', '__ns:=' + self.vision.bot.NAME, 
                'image:=ardrone/image_raw'])
        
    def send_target(self, frame, rectangle):
        '''Sends a target to the OpenTLD module'''
        self.target.bb.x = rectangle[0]
        self.target.bb.y = rectangle[1]
        self.target.bb.width = rectangle[2] - rectangle[0]
        self.target.bb.height = rectangle[3] - rectangle[1]
        self.target.bb.confidence = 1.0
        self.target.img = frame
        self.pub_target.publish(self.target)

    def callback(self, box):
        '''Callback function for the found box'''
        self.tracked_target = (box.x, box.y, box.x + box.width, box.y + \
                               box.height)
        self.navigate(box)
        
    def get_target(self):
        '''Returns the tracked target'''
        if self.tracked_target:
            return self.tracked_target
        return False
        
    def navigate(self, box):
        '''Navigate towards the box'''
        if box.confidence < 0.3:
            return
        self.time_last_detection = time.time()
        speed = 0.1
        offset = 20
        
        center_box = (box.x + box.width / 2, box.y + box.height / 2)
        center = (320, 160)
        
        # Turn control
        if abs(center[0] - center_box[0]) > offset:
            self.controls['angularz'] = speed * ((center[0] - center_box[0]) / float(center[0]))
        else:
            self.controls['angularz'] = 0
                
        # Forward/Backward control
        self.controls['linearx'] = speed * ((self.target.bb.width - \
                                   box.width) / float(self.target.bb.width))
                
        # Height control
        if abs(center_box[1] - center[1]) > offset:
            self.controls['linearz'] = speed * ((center[1] - center_box[1]) \
                                       / float(center[1]))
        else:
            self.controls['linearz'] = 0
            
    def get_controls(self):
        '''Returns the controls of OpenTLD'''
        if self.time_last_detection - time.time() > 2:
            return False
        return self.controls
            
class FollowLine(threading.Thread):
    '''This class implements a line-following algorithm'''
    
    def __init__(self, vision):
        '''Constructor of Follow Line'''
        super(FollowLine, self).__init__()
        self.vision = vision
        self.running = True
        self.controls = dict()
        self.mean_x = 0
        self.mean_y = 0
        self.max_x = 0
        self.max_y = 0
        self.speed_x = 0
        self.speed_y = 0
        self.turn = 0
        self.center = (0, 0)
        
        weight_column = np.array([[i**4] for i in xrange(360 * 2, 0, -1)])
        self.weight_matrix = np.repeat(weight_column, 1280, 1)
        self.weight_circle = np.zeros((360, 640))
        self.inverted_weight_circle = np.zeros((360, 640))
        self.x_matrix = np.zeros((360, 640))
        self.y_matrix = np.zeros((360, 640))
        for x in xrange(640):
            for y in xrange(360):
                self.x_matrix[y][x] = x
                self.y_matrix[y][x] = y
                dist = ((x - 320) ** 2 + (y - 180) ** 2) ** 0.5
                self.weight_circle[y][x] = dist ** 2
                self.inverted_weight_circle[y][x] = 1.0 / ((dist+0.1) ** 4)
        
        # If our rotation was to the left for the drone, this corresponds
        # to a positive rotation in algebra
        self.rotation_matrices = dict()
        for turn in xrange(-180, 180, 30):
            rotation_matrix = interpolation.rotate(self.weight_matrix,
                turn, reshape=False)
            smaller_weight_matrix = rotation_matrix[180:-180, 320:-320]
            self.rotation_matrices[turn] = smaller_weight_matrix

    def shutdown(self):
        '''Shuts down the follow line program'''
        self.running = False
        
    def is_active(self):
        '''Check if the algorithm is active'''
        return self.running

    def get_rot_matrix(self, turn):
        '''Returns a rotation matrix for the turn'''
        for t in xrange(-180, 180, 30):
            if turn >= t:
                return self.rotation_matrices[t]
                
    def get_angle(self, a1, a2):
        '''Returns a angle'''
        return math.atan2(a2[1]-a1[1], a2[0]-a1[0]) + 0.5 * 3.1415926        
    
    def run(self):
        '''Main function, which calculates the controls based on the line'''
        while self.running:
            # Good vals
            #hsv_min = cv.Scalar(96, 104, 0, 0)
            #hsv_max = cv.Scalar(120, 255, 255, 0)
            frame = self.vision.get_frame(encoding=None, hsv_filter=True)
            if not frame:
                continue

            self.center = (frame.width / 2, frame.height / 2)

            matrix = cv.GetMat(frame)
            filtered = np.asarray(matrix)
            filtered = filtered * (filtered > 180) / 255.0

            # Left turn -> z negative, front -> x positive,
            # right -> y positive
            if self.speed_y == 0 and self.speed_x == 0:
                old_turn = 0
            else:
                old_turn = math.atan2(self.speed_y, self.speed_x)
            old_turn_in_degrees = old_turn * 57.2957795
            self.turn = 0 * self.turn + old_turn_in_degrees * 1

            weighted = filtered * self.get_rot_matrix(self.turn) * \
                       self.weight_circle

            # Calculate point on line closest to the drone
            weighted2 = filtered * self.inverted_weight_circle
            self.max_y, self.max_x = np.unravel_index(weighted2.argmax(),
                                                      weighted2.shape)

            # Line lost            
            if np.sum(weighted) == 0:
                self.speed_x *= 0.9
                self.speed_y *= 0.9
                self.controls = {
                    'linearx': self.speed_x,
                    'lineary': self.speed_y
                }
                continue
    
#            cv.ShowImage("Weighted",
#                          cv.fromarray(weighted / np.max(weighted)))
    
            weighted *= (1.0 / np.sum(weighted))
    
            self.mean_x = int(np.sum(weighted * self.x_matrix))
            self.mean_y = int(np.sum(weighted * self.y_matrix))
    
    
#            cv.Line(image, center, (mean_x, mean_y),cv.CV_RGB(0, 255, 0), 3,
#                    8)
            self.turn = self.get_angle(self.center,
                                       (self.mean_x,  self.mean_y))
            # turn is an angle towards the next best point
            # find an x and y that go to this point, given that the total
            #speed=0.1
            # cos(turn) = x / 0.1
            # sin(turn) = y / 0.1
    
            self.speed_x = 0.025 * math.cos(self.turn)
            self.speed_y = -0.025 * math.sin(self.turn)
    
            self.controls = {
                'linearx': self.speed_x,
                'lineary': self.speed_y,
                'angularz': -self.turn * 0
            }
                
    def get_controls(self):
        '''Returns the controls'''
        return self.controls
    
    def get_mean(self):
        '''Returns the mean'''
        return (self.mean_x, self.mean_y)
        
    def get_max(self):
        '''Returns the max'''
        return (self.max_x, self.max_y)
        
    def get_center(self):
        '''Returns the center of the frame'''
        return self.center