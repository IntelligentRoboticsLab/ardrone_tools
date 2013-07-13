'''This file implements all the subframes of the application. Handles user
interface events
'''
import wx
import rosbag
import rospy
import subprocess

from panels import *

class RecorderFrame(wx.Frame):
    '''Class implementing the recorder frame'''

    def __init__(self, parent=None, id=-1, title='Recorder'):
        '''Constructor of the RecorderFrame, which initializes the frame.'''
        assert parent != None
        super(RecorderFrame, self).__init__(parent=parent, id=id,
                                            title=title)
        
        self.recording = False
        self.recording_path = None
        self.recording_topics = None
        self.selected_topics = None
        self.recorder = None
        
        self.create_ui()
        self.Centre()
        
    def create_ui(self):
        '''Create User Interface'''
        self.main_window = wx.BoxSizer(wx.VERTICAL)
        
        # Button
        self.control_window = wx.BoxSizer(wx.HORIZONTAL)
        self.record_button = wx.Button(self, label='Record')
        self.record_button.Bind(wx.EVT_BUTTON, self.on_record)
        self.stop_button = wx.Button(self, label='Stop')
        self.stop_button.Bind(wx.EVT_BUTTON, self.on_stop)

        self.control_window.Add(self.record_button)
        self.control_window.Add(self.stop_button)
        
        # Text
        self.status = wx.StaticText(self, label='File: None\nTopics: None')

        # Close event management
        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.main_window.Add(self.control_window, flag=wx.ALIGN_CENTER, border=1)
        self.main_window.Add(self.status, border=2)
        self.SetSizerAndFit(self.main_window)
        
    def update_file_information(self):
        '''Updates the file information shown'''
        status_message = ''
        if self.recorder:
            status_message = 'File: %s\nTopics:\n' % (self.recording_path)
            for topic in self.selected_topics:
                status_message += topic + '\n'
        else:
            status_message = 'File: None\nTopics: None' 
        self.status.SetLabel(status_message)
        self.Fit()
        self.Center()
        self.Layout()
        
    def on_record(self, e):
        '''Handles record button events'''
        if self.recorder:
            self.recorder.send_signal(subprocess.signal.SIGINT)
            self.recorder = None
        wildcard = "Rosbag file (*.bag)|*.bag"
        file_dialog = wx.FileDialog(self, message="Save a rosbag file",
            wildcard=wildcard, style=wx.SAVE | wx.CHANGE_DIR)
        if file_dialog.ShowModal() == wx.ID_OK:
            self.recording_path = file_dialog.GetPath()            
            file_dialog.Destroy()
            
            self.recording_topics = sorted(set(x[0] for x in rospy.get_published_topics()))
            self.recording_topics.insert(0, 'All topics')
            
            multi_dialog = wx.MultiChoiceDialog(self, "Pick topics for the rosbag",
                "Choose topics", self.recording_topics)
            if multi_dialog.ShowModal() == wx.ID_OK:
                selections = multi_dialog.GetSelections()
                if selections:
                    if 0 in selections:
                        self.selected_topics = self.recording_topics[1:]
                    else:
                        self.selected_topics = [self.recording_topics[x] for x in selections]
                    self.recorder = subprocess.Popen(['rosrun', 'rosbag',
                        'record', '-O', self.recording_path] + self.selected_topics)
                    self.update_file_information()
            multi_dialog.Destroy()
        else:    
            file_dialog.Destroy()
    
    def on_stop(self, e):
        '''Handles stop button events'''
        if self.recorder:
            self.recorder.send_signal(subprocess.signal.SIGINT)
            self.recorder = None
            self.update_file_information()
        
    def on_close(self, e):
        '''Handles closing events'''
        if self.recorder:
            self.recorder.send_signal(subprocess.signal.SIGINT)
        self.Destroy()


class PlayerFrame(wx.Frame):
    '''Class implementing the player frame'''
    
    def __init__(self, parent=None, id=-1, title='Player'):
        '''Constructor of the PlayerFrame, which initializes the frame.'''
        assert parent != None
        super(PlayerFrame, self).__init__(parent=parent, id=id, title=title)
        
        self.recording = False
        self.recording_path = None
        self.recording_topics = None
        self.selected_topics = None
        self.player = None
        
        self.create_ui()
        self.Centre()
        
    def create_ui(self):
        '''Create User Interface'''
        self.main_window = wx.BoxSizer(wx.VERTICAL)
        
        # Button
        self.control_window = wx.BoxSizer(wx.HORIZONTAL)
        self.open_button = wx.Button(self, label='Open')
        self.open_button.Bind(wx.EVT_BUTTON, self.on_open)
        self.play_button = wx.Button(self, label='Play')
        self.play_button.Bind(wx.EVT_BUTTON, self.on_play)
        self.stop_button = wx.Button(self, label='Stop')
        self.stop_button.Bind(wx.EVT_BUTTON, self.on_stop)
        self.change_topics_button = wx.Button(self, label='Change topics')
        self.change_topics_button.Bind(wx.EVT_BUTTON, self.on_change_topics)

        self.control_window.Add(self.open_button)
        self.control_window.Add(self.play_button)
        self.control_window.Add(self.stop_button)
        self.control_window.Add(self.change_topics_button)
        
        # Text
        self.status = wx.StaticText(self, label='File: None\nTopics: None')

        # Close event management
        self.Bind(wx.EVT_CLOSE, self.on_close)

        self.main_window.Add(self.control_window, flag=wx.ALIGN_CENTER, border=1)
        self.main_window.Add(self.status, border=2)
        self.SetSizerAndFit(self.main_window)
        
    def update_file_information(self):
        '''Updates the file information shown'''
        status_message = 'Path: %s\nTopics:\n' % (self.recording_path)
        for topic in self.selected_topics:
            status_message += topic + '\n'
        self.status.SetLabel(status_message)
        self.Fit()
        self.Center()
        self.Layout()
    
    def on_open(self, e):
        '''Handles open button events'''
        self.recording = False
        self.recording_path = None
        self.recording_topics = None
        self.selected_topics = None
        self.status.SetLabel('File: None\nTopics: None')
        
        wildcard = "Rosbag file (*.bag)|*.bag|All files (*.*)|*.*"
        file_dialog = wx.FileDialog(self, message="Choose a rosbag file",
            wildcard=wildcard, style=wx.OPEN | wx.CHANGE_DIR)
            
        if file_dialog.ShowModal() == wx.ID_OK:
            self.recording_path = file_dialog.GetPath()
            file_dialog.Destroy()
            
            bag = rosbag.Bag(self.recording_path)
            self.recording_topics = sorted(set([c.topic for c in bag._get_connections()]))
            self.recording_topics.insert(0, 'All topics')
            bag.close()
            
            multi_dialog = wx.MultiChoiceDialog(self, "Pick topics of the rosbag",
                "Choose topics", self.recording_topics)
            if multi_dialog.ShowModal() == wx.ID_OK:
                selections = multi_dialog.GetSelections()
                if selections:
                    if 0 in selections:
                        self.selected_topics = self.recording_topics[1:]
                    else:
                        self.selected_topics = [self.recording_topics[x] for x in selections]
                    self.recording = True
            multi_dialog.Destroy()
        else:    
            file_dialog.Destroy()

        if self.recording:
            self.update_file_information()
        e.Skip()
        
    def on_play(self, e):
        '''Handles play button events'''
        if self.recording:
            if self.player:
                self.player.kill()            
            name = ''
            for topic in self.selected_topics:
                pos = topic.find('/ardrone/')
                if pos >= 0:
                    name = topic[:pos]
                    break   
            self.Parent.replay_drone(name)
            self.player = subprocess.Popen(['rosrun', 'rosbag', 'play', self.recording_path,
                '--topics'] + self.selected_topics)
            
        e.Skip()
            

    def on_stop(self, e):
        '''Handles stop button events'''
        if self.player:
            self.player.kill()
        e.Skip()
        
    def on_change_topics(self, e):
        '''Handles change topics button events'''
        if self.recording:
            multi_dialog = wx.MultiChoiceDialog(self, "Change topics of the rosbag",
                "Change topics", self.recording_topics)
            selections = [self.recording_topics.index(topic) for topic in self.selected_topics]
            multi_dialog.SetSelections(selections)
            if multi_dialog.ShowModal() == wx.ID_OK:
                selections = multi_dialog.GetSelections()
                if selections:
                    if 0 in selections:
                        self.selected_topics = self.recording_topics[1:]
                    else:
                        self.selected_topics = [self.recording_topics[x] for x in selections]
            multi_dialog.Destroy()
            self.update_file_information()
        e.Skip()
        
    def on_close(self, e):
        '''Handles closing events'''
        if self.player:
            self.player.kill()
        self.Destroy()

         
class ColourCalibrationFrame(wx.Frame):
    '''Class implementing the colour calibration frame'''
    
    def __init__(self, parent=None, id=-1, title='Colour Calibration'):
        '''Constructor of the ColourCalibrationFrame, which initializes the frame.'''
        assert parent != None
        super(ColourCalibrationFrame, self).__init__(parent=parent, id=id, title=title)
        self.create_ui()
        self.Centre()
    
    def create_ui(self):
        '''Create User Interface'''
        self.main_window = wx.GridBagSizer(3, 1)
        
        # Frame
        self.frame = wx.Panel(self, wx.ID_ANY, style=wx.NO_BORDER, size=(640, 320))
        self.empty_image = wx.EmptyImage(640, 320)
        self.bmp = wx.StaticBitmap(parent=self.frame, id=wx.ID_ANY, 
            bitmap= wx.BitmapFromImage(self.empty_image))
        self.video_timer = wx.Timer(self)
        self.video_timer.Start(1000/30)
        self.Bind(wx.EVT_TIMER, self.on_video_frame, self.video_timer)
        
        # Sliders
        self.sliders = wx.GridBagSizer(6, 2)
        
        min_hue_txt = wx.StaticText(self)
        min_hue_txt.SetLabel('Minimum Hue')
        self.slider_min_hue = wx.Slider(self, wx.ID_ANY, value=0,
            minValue=0, maxValue=180, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
        max_hue_txt = wx.StaticText(self)
        max_hue_txt.SetLabel('Maximum Hue')        
        self.slider_max_hue = wx.Slider(self, wx.ID_ANY, value=180,
            minValue=0, maxValue=180, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
            
        min_saturation_txt = wx.StaticText(self)
        min_saturation_txt.SetLabel('Minimum Saturation')
        self.slider_min_saturation = wx.Slider(self, wx.ID_ANY, value=0,
            minValue=0, maxValue=255, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
        max_saturation_txt = wx.StaticText(self)
        max_saturation_txt.SetLabel('Maximum Saturation')
        self.slider_max_saturation = wx.Slider(self, wx.ID_ANY, value=255,
            minValue=0, maxValue=255, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
            
        min_value_txt = wx.StaticText(self)
        min_value_txt.SetLabel('Minimum Value')
        self.slider_min_value = wx.Slider(self, wx.ID_ANY, value=0,
            minValue=0, maxValue=255, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
        max_value_txt = wx.StaticText(self)
        max_value_txt.SetLabel('Maximum Value')
        self.slider_max_value = wx.Slider(self, wx.ID_ANY, value=255,
            minValue=0, maxValue=255, size=(200,50), style=wx.SL_HORIZONTAL |
            wx.SL_LABELS | wx.SL_AUTOTICKS)
                    
        self.sliders.Add(min_hue_txt, (0, 0))
        self.sliders.Add(self.slider_min_hue, (0, 1))
        self.sliders.Add(max_hue_txt, (1, 0))
        self.sliders.Add(self.slider_max_hue, (1, 1))
        self.sliders.Add(min_saturation_txt, (2, 0))
        self.sliders.Add(self.slider_min_saturation, (2, 1))
        self.sliders.Add(max_saturation_txt, (3, 0))
        self.sliders.Add(self.slider_max_saturation, (3, 1))
        self.sliders.Add(min_value_txt, (4, 0))
        self.sliders.Add(self.slider_min_value, (4, 1))
        self.sliders.Add(max_value_txt, (5, 0))
        self.sliders.Add(self.slider_max_value, (5, 1))
        self.sliders.Layout()

        # Button
        self.button = wx.Button(self, label='OK')
        self.button.Bind(wx.EVT_BUTTON, self.on_click)
        
        # Setup window
        self.main_window.Add(self.frame, (0, 0))
        self.main_window.Add(self.sliders, (1, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        self.main_window.Add(self.button, (2, 0), flag=wx.ALIGN_CENTER_HORIZONTAL)
        
        # Close event management
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.SetAutoLayout(True)
        self.SetSizer(self.main_window)
        self.Fit()
        self.Layout()
    
    def on_video_frame(self, e):
        '''Handles video events'''
        if self.Parent.drone:
            hsv_min = (self.slider_min_hue.GetValue(),
                       self.slider_min_saturation.GetValue(),
                       self.slider_min_value.GetValue(), 0)
            hsv_max = (self.slider_max_hue.GetValue(),
                       self.slider_max_saturation.GetValue(),
                       self.slider_max_value.GetValue(), 0)
            self.Parent.drone.cv.set_hsv_filter(hsv_min, hsv_max)
            frame = self.Parent.drone.cv.get_frame(encoding='rgb8', hsv_filter=True) 
            if frame:
                new_frame = wx.EmptyImage(frame.width, frame.height)
                new_frame.SetData(frame.tostring())
                self.bmp.SetBitmap(wx.BitmapFromImage(new_frame))
        else:
            self.bmp.SetBitmap(wx.BitmapFromImage(self.empty_image))
        e.Skip()
            
    def on_click(self, e):
        '''Handles click events'''
        if self.Parent:
            if self.Parent.drone:
                hsv_min = (self.slider_min_hue.GetValue(),
                           self.slider_min_saturation.GetValue(),
                           self.slider_min_value.GetValue(), 0)
                hsv_max = (self.slider_max_hue.GetValue(),
                           self.slider_max_saturation.GetValue(),
                           self.slider_max_value.GetValue(), 0)
                self.Parent.drone.cv.set_hsv_filter(hsv_min, hsv_max)
        self.Close()
        e.Skip()
        
    def on_close(self, e):
        '''Handles closing events'''
        self.Parent.video_timer.Start(1000/30)
        self.Destroy()