'''This file contains the main layout of a bot.
'''

class Bot():
    '''Class implementing a bot'''

    def __init__(self):
        '''Main variables of a bot'''
        self.NAME       = None     # Name of the bot
        self.CAMERA     = None     # List of cameras
        self.CAMERA_ID  = None     # Camera
        self.NAVDATA    = None     # Navigation data
        self.IMU        = None     # IMU data
        self.CONTROLLER = None     # Controller controlling the bot
        self.CONTROLS   = None     # Controls for the bot
