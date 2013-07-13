''' This file implements the main frame of the graphical user interface and
implements all the functionality.
'''
import rospy
import wx

from ..bots.ardrone import ARDrone
from .frames import *

class MainFrame(wx.Frame):
    '''Class implementing the main frame'''
    
    def __init__(self, app, parent=None, id=-1,
                 title='Drone Control Interface'):
        '''Initializes the main application window'''
        # ROS Instantiation
        rospy.init_node('GUI')
        
        # Initialize super class
        super(MainFrame, self).__init__(parent=parent, id=id, title=title,
                                        style=wx.DEFAULT_FRAME_STYLE ^
                                        wx.RESIZE_BORDER)
        
        # Application, catching key events
        self.app = app
        self.app.Bind(wx.EVT_KEY_DOWN, self.on_keydown)
        self.app.Bind(wx.EVT_KEY_UP, self.on_keyup)

        # Menu bar
        self.menu_bar = self.create_menu()
        self.SetMenuBar(self.menu_bar)
        
        # Control variables
        self.drones = list()
        self.drone = None
        self.speed = 0.2
        self.multiple_control = False

        self.create_ui()
        
        self.Fit()
        self.focus()
        self.Centre()
        self.Show()
        
    # UI FUNCTIONALITY
    def create_ui(self):
        '''Creates the user interface'''
        self.main_window = wx.BoxSizer(wx.VERTICAL)
        self.top_window = wx.BoxSizer(wx.HORIZONTAL)
        self.left_window = wx.GridBagSizer(7, 1)
        self.bottom_window = wx.BoxSizer(wx.HORIZONTAL)
        
        # Fixed sizes for now
        self.left_panel = wx.Panel(self, wx.ID_ANY, style=wx.NO_BORDER, size=(160, 320))
        self.right_panel = wx.Panel(self, wx.ID_ANY, style=wx.NO_BORDER, size=(640, 320))
        self.bottom_panel = wx.Panel(self, wx.ID_ANY, style=wx.NO_BORDER, size=(800, 180))

        # Colours for debugging
        #self.left_panel.SetBackgroundColour(wx.GREEN)
        #self.right_panel.SetBackgroundColour(wx.RED)
        #self.bottom_panel.SetBackgroundColour(wx.BLUE)    

        self.top_window.Add(self.left_panel, 0, wx.EXPAND)
        self.top_window.Add(self.right_panel, 0, wx.EXPAND)
        self.main_window.Add(self.top_window, 0, wx.EXPAND)
        self.main_window.Add(self.bottom_panel, 0, wx.EXPAND)

        # Settings for main window
        self.SetSizer(self.main_window)
        self.Layout()
        
        # Settings for subwindows
        self.bottom_panel.SetSizer(self.bottom_window)

        # User interface information
        self.ui_info = dict()
        self.ui_info['State'] = wx.StaticText(self.left_panel)
        self.ui_info['State'].SetLabel('State: Unknown')
        self.ui_info['Controlling'] = wx.StaticText(self.left_panel)
        self.ui_info['Controlling'].SetLabel('Controlling: Unknown')
        self.ui_info['Battery'] = wx.StaticText(self.left_panel)
        self.ui_info['Battery'].SetLabel('Battery: Unknown')
        self.ui_info['Altitude'] = wx.StaticText(self.left_panel)
        self.ui_info['Altitude'].SetLabel('Altitude: Unknown')
        self.ui_info['Control'] = wx.StaticText(self.left_panel)
        self.ui_info['Control'].SetLabel('Control: Unknown')
        
        self.drone_selector = wx.ComboBox(self.left_panel, size=(self.left_panel.Size[0], 29), value='None', choices=[], style=wx.CB_READONLY)
        self.drone_selector.Bind(wx.EVT_COMBOBOX, self.on_drone_selection)
        
        self.stop_button = wx.Button(self.left_panel, label='Stop')
        self.stop_button.Bind(wx.EVT_BUTTON, self.on_stop)
        
        self.focus_button = wx.Button(self.left_panel, label='FOCUS')

        self.left_window.Add(self.drone_selector, (0, 0), flag=wx.EXPAND)
        self.left_window.Add(self.ui_info['State'], (1, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_window.Add(self.ui_info['Control'], (2, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_window.Add(self.ui_info['Battery'], (3, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_window.Add(self.ui_info['Altitude'], (4, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_window.Add(self.stop_button, (5, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_window.Add(self.focus_button, (6, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.left_panel.SetSizer(self.left_window)
        self.left_panel.SetAutoLayout(True)
        self.left_window.Layout()
        
        # Set timer for ui updates
        self.ui_timer = wx.Timer(self)
        self.ui_timer.Start(1000/10)
        self.Bind(wx.EVT_TIMER, self.on_ui_info, self.ui_timer)

        # Create videostream
        self.create_video_stream()
        
        # Set timer for navigation updates
        self.navigation_timer = wx.Timer(self)
        self.navigation_timer.Start(1000/10)
        self.Bind(wx.EVT_TIMER, self.on_orders, self.navigation_timer)
        
        # Close event management
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
    def focus(self):
        '''Focuses mouse and keyboard focus on the focus button'''
        self.focus_button.SetFocus()

    def create_menu(self):
        '''Creates the top menu bar and its functionality'''
        menu_bar = wx.MenuBar()
        
        # File menu        
        file_menu = wx.Menu()
        file_menu_player = file_menu.Append(wx.ID_ANY, 'Player',
                                            'Open player')
        file_menu_recorder = file_menu.Append(wx.ID_ANY, 'Recorder',
                                              'Open recorder')
        file_menu.AppendSeparator()
        file_menu_quit = file_menu.Append(wx.ID_EXIT, 'Quit',
                                          'Quit application')
        menu_bar.Append(file_menu, '&File')
        
        # AR.Drone menu
        drone_menu = wx.Menu()
        drone_menu_connect = drone_menu.Append(wx.ID_ANY, 'Connect',
                                               'Connect drone')
        drone_menu_disconnect = drone_menu.Append(wx.ID_ANY, 'Disconnect',
                                                  'Disconnect drone')
        menu_bar.Append(drone_menu, '&Drone')
        
        # Calibration menu
        calibration_menu = wx.Menu()
        calibration_menu_colours = calibration_menu.Append(wx.ID_ANY,
                                                           'Colours',
                                                           'Calibrate \
                                                           colours')
        menu_bar.Append(calibration_menu, '&Calibrate')
        
        self.Bind(wx.EVT_MENU, self.on_player, file_menu_player)
        self.Bind(wx.EVT_MENU, self.on_record, file_menu_recorder)
        self.Bind(wx.EVT_MENU, self.on_quit, file_menu_quit)
        self.Bind(wx.EVT_MENU, self.on_connect, drone_menu_connect)
        self.Bind(wx.EVT_MENU, self.on_disconnect, drone_menu_disconnect)
        self.Bind(wx.EVT_MENU, self.on_calibration_colours, 
                  calibration_menu_colours)
        
        return menu_bar
                
    def create_video_stream(self):
        '''Creates the main window and the thumbnails'''
        self.empty_image = wx.EmptyImage(640, 320)
        self.bmp = wx.StaticBitmap(parent=self.right_panel, id=wx.ID_ANY,
                                   bitmap=wx.BitmapFromImage( \
                                   self.empty_image))
                
        self.video_timer = wx.Timer(self)
        self.video_timer.Start(1000/30)
        self.Bind(wx.EVT_TIMER, self.on_video_frame, self.video_timer)
        
    # FUNCTIONALITY
    def switch_speed(self, speed):
        '''Changes the speed of manual control of the drone'''
        new_speed = self.speed + speed
        if new_speed >= -1 and new_speed <= 1:
            self.speed = new_speed
        print "The current speed is:", self.speed

    def connect_drone(self, ip):
        '''Connects to a drone with the given ip'''
        drone_id = 0
        drone_name = '/drone' + str(drone_id)
        
        invalid = True
        while(invalid):
            invalid = False
            for drone in self.drones:
                if drone.NAME == drone_name:
                    drone_id += 1
                    drone_name = '/drone' + str(drone_id)
                    invalid = True

        self.drones.append(ARDrone(name=drone_name, ip=ip))
        self.drone_selector.Append(drone_name + ' ' + ip)
        if len(self.drones) == 1:
            self.drone_selector.SetSelection(0)
            self.drone = self.drones[0]
            
    def replay_drone(self, name):
        '''Creates a replay drone, which replays a dataset'''
        names = self.drone_selector.GetItems()
        if '/drone replay' in names:
            id = names.index('/drone replay')
            self.disconnect_drone(id)
            self.drones.insert(id, ARDrone(name=name, recording=True))
        else:
            self.drones.append(ARDrone(name=name, recording=True))
        
        self.drone_selector.Append('/drone replay')
        if len(self.drones) == 1:
            self.drone_selector.SetSelection(0)
            self.drone = self.drones[0]
        
    def disconnect_drone(self, id):
        '''Disconnects a drone given the id'''
        if id >= 0:
            if self.drone and self.drone.UID == self.drones[id].UID:
                self.drone_selector.SetValue('None')
                self.drone = None
            self.drones[id].shutdown()
            self.drone_selector.Delete(id)
            del self.drones[id]

    # EVENT MANAGEMENT    
    def on_quit(self, e):
        '''Close application when user quits the application'''
        self.Close()
        
    def on_stop(self, e):
        '''Emergency button, which stops all the drone'''
        for drone in self.drones:
            drone.switch_off_autonomy()
            drone.land()
            
    def on_multiple_control(self, e):
        '''Checkbox to allow multiple drone control'''
        self.multiple_control = e.GetSelection()
        self.focus()
        
    def on_video_frame(self, e):
        '''Updates the video feed'''
        if self.drone:
            frame = self.drone.cv.get_frame(encoding='rgb8')
            if frame:
                new_frame = wx.EmptyImage(frame.width, frame.height)
                new_frame.SetData(frame.tostring())
                self.bmp.SetBitmap(wx.BitmapFromImage(new_frame))
        else:
            self.bmp.SetBitmap(wx.BitmapFromImage(self.empty_image))

        e.Skip()

    '''
    # For other way of drawing which is currently not working.
    def on_paint(self, e):
        if self.bmp:
            wx.BufferedPaintDC(self.right_panel, self.bmp)
        e.Skip()
    
    def on_video_frame(self, e):
        if self.drone:
            raw_image = self.drone.get_cv_frame_rgb()
            if raw_image:
                self.bmp.CopyFromBuffer()
                new_image.SetData(raw_image.tostring())
                self.image.SetBitmap(wx.BitmapFromImage(new_image))
        e.Skip()    
    '''
        
    def on_ui_info(self, e):
        '''Updates the user information of the drone'''
        navdata = None
        if self.drone:
            navdata = self.drone.get_navdata()
            
        if navdata:
            self.ui_info['State'].SetLabel('State: %s' % \
                                           (self.drone.get_state()))
            self.ui_info['Control'].SetLabel('Control: %s' % \
                                             ('Autonomous' if \
                                             self.drone.get_autonomy() else \
                                             'Manual'))
            self.ui_info['Battery'].SetLabel('Battery: %.2f %%' % \
                                             (navdata.batteryPercent))
            self.ui_info['Altitude'].SetLabel('Altitude: %.0f mm' % \
                                              (navdata.altd))
        else:
            for item in self.ui_info:
                self.ui_info[item].SetLabel(item + ': Unknown')
                
        self.left_window.Layout()
        e.Skip()
            
    def on_orders(self, e):
        '''Sends orders to the drones'''
        if self.multiple_control:
            for drone in self.drones:
                drone.move()
        else:
            if self.drone:
                self.drone.move()
                
    def on_close(self, e):
        '''Shuts down all the functionality of the application'''
        self.app.Unbind(wx.EVT_KEY_DOWN)
        self.app.Unbind(wx.EVT_KEY_UP)
        self.Unbind(wx.EVT_TIMER)
        for drone in self.drones:
            drone.shutdown()
        self.Destroy()

    def on_calibration_colours(self, e):
        '''Opens the calibration frame'''
        self.video_timer.Start(1000/5)
        colour_calibration_frame = ColourCalibrationFrame(parent=self)
        colour_calibration_frame.Show()
        self.focus()

    def on_player(self, e):
        '''Opens the dataset player frame'''
        player_frame = PlayerFrame(parent=self)
        player_frame.Show()
        self.focus()
        
    def on_record(self, e):
        '''Opens the dataset recorder frame'''
        recorder_frame = RecorderFrame(parent=self)
        recorder_frame.Show()
        self.focus()
        
    def on_connect(self, e):
        '''Opens a dialog to connect'''
        dialog = wx.TextEntryDialog(self, 'Please fill in the IP address:',
            'Connect drone')
        dialog.SetValue('192.168.1.1')
        if dialog.ShowModal() == wx.ID_OK:
            ip = dialog.GetValue()
            self.connect_drone(ip)
        self.SetFocus()
        dialog.Destroy()
        self.focus()
    
    def on_disconnect(self, e):
        '''Opens a dialog to disconnect'''
        choices = self.drone_selector.GetItems()
        if not choices:
            wx.MessageBox('No drones connected', 'Error', wx.ICON_ERROR)
            return
        dialog = wx.SingleChoiceDialog(self, 'Please select the drone you would like to disconnect:',
            'Disconnect drone', choices=choices)
        if dialog.ShowModal() == wx.ID_OK:
            id = dialog.GetSelection()
            self.disconnect_drone(id)
        dialog.Destroy()
        self.focus()
        
    def on_drone_selection(self, e):
        '''Selects the drone that is selected'''
        self.drone = self.drones[e.GetSelection()]
        
    def on_keydown(self, e):
        '''Handles keydown events'''
        key = e.GetKeyCode()
        if self.multiple_control:
            for drone in self.drones:
                self.send_keydown(drone, key)
        else:
            if self.drone == None or self.drone and self.drone.get_autonomy():
                pass
            else:
                self.send_keydown(self.drone, key)                
        e.Skip()
        
    def send_keydown(self, drone, key):
        '''Sends the keydown events'''
        if drone.get_autonomy():
            pass
        elif key == 315: # up
            drone.set_control(linearx = self.speed)
        elif key == 314: # left
            drone.set_control(lineary = self.speed)
        elif key == 317: # down
            drone.set_control(linearx = -self.speed)
        elif key == 316: # right
            drone.set_control(lineary = -self.speed)
        elif key == 87: # w
            drone.set_control(linearz = self.speed)
        elif key == 65: # a
            drone.set_control(angularz = self.speed)
        elif key == 83: # s
            drone.set_control(linearz = -self.speed)
        elif key == 68: # d
            drone.set_control(angularz = -self.speed)
        elif key == 67: # c
            drone.toggle_cam()
        elif key == 82: # r 
            drone.reset()
        elif key == 84: # t
            drone.flat_trim()
        elif key == 45: # -
            self.switch_speed(-0.05)
        elif key == 61: # =
            self.switch_speed(0.05)
        elif key == 32: # space
            drone.toggle_airborne()

    def on_keyup(self, e):
        '''Handles keyup events'''
        key = e.GetKeyCode()
        if self.multiple_control:
            for drone in self.drones:
                self.send_keyup(drone, key)
        else:
            if self.drone == None:
                pass
            else:
                self.send_keyup(self.drone, key)                
            
    def send_keyup(self, drone, key):
        '''Sends keyup events'''
        if drone.get_autonomy():
            pass
        elif key == 315: # up
            drone.set_control(linearx = 0)
        elif key == 314: # left
            drone.set_control(lineary = 0)
        elif key == 317: # down
            drone.set_control(linearx = 0)
        elif key == 316: # right
            drone.set_control(lineary = 0)
        elif key == 87: # w
            drone.set_control(linearz = 0)
        elif key == 65: # a
            drone.set_control(angularz = 0)
        elif key == 83: # s
            drone.set_control(linearz = 0)
        elif key == 68: # d
            drone.set_control(angularz = 0)