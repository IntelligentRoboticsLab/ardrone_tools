'''Implements special user interface element features.
'''
import wx

class SelectStaticBitmap(wx.StaticBitmap):
    '''Implements a bitmap on which a rectangle is drawn when selecting'''
    
    def __init__(self, *args, **kw):
        '''Constructor of the bitmap'''
        wx.StaticBitmap.__init__(self, *args, **kw)

        self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
        self.Bind(wx.EVT_LEFT_UP, self.on_left_up)
        self.Bind(wx.EVT_MOTION, self.on_mouse_move)

        self.selecting = False
        self.start_pos = None
        self.end_pos = None
        self.overlay = wx.Overlay()
        
    def draw_box(self, box):
        '''Draws a box onto the bitmap'''
        if box[0] == box[2] or box[1] == box[3]:
            return
        rect = wx.RectPP((box[0], box[1]), (box[2], box[3]))
            
        # Draw the rubber-band rectangle using an overlay so it 
        # will manage keeping the rectangle and the former window 
        # contents separate. 
        dc = wx.ClientDC(self) 
        odc = wx.DCOverlay(self.overlay, dc) 
        odc.Clear() 
            
        dc.SetPen(wx.Pen("blue", 2)) 
        dc.SetBrush(wx.TRANSPARENT_BRUSH) 
        dc.DrawRectangleRect(rect) 

        del odc # work around a bug in the Python wrappers to make 
                # sure the odc is destroyed before the dc is. 
                
    def is_selecting(self):
        '''Check if someone is selecting'''
        return self.selecting

    def get_select_box(self):
        '''Returns the select box if selected'''
        if self.start_pos and self.end_pos:
            x0, y0 = self.start_pos
            x1, y1 = self.end_pos
            left_top_x = x0
            left_top_y = y0
            right_bottom_x = x1
            right_bottom_y = y1
            
            if x1 < x0:
                left_top_x = x1
                right_bottom_x = x0

            if y1 < y0:
                left_top_y = y0
                right_bottom_y = y1
            
            return (left_top_x, left_top_y, right_bottom_x, right_bottom_y)
        else:
            return False
            
    def on_left_down(self, e):
        '''Handles left mouse button down events'''
        self.selecting = True
        self.start_pos = e.GetPosition()
        self.end_pos = None

    def on_mouse_move(self, e):
        '''Handes mouse movement events'''
        if e.LeftIsDown(): 
            pos = e.GetPosition()
            self.draw_box((self.start_pos[0], self.start_pos[1], pos[0], pos[1]))
                
    def on_left_up(self, e):
        '''Handles left mouse buttons up events'''
        self.selecting = False
        self.end_pos = e.GetPosition()
        # When the mouse is released we reset the overlay and it 
        # restores the former content to the window.
        dc = wx.ClientDC(self) 
        odc = wx.DCOverlay(self.overlay, dc) 
        odc.Clear() 
        del odc 
        self.overlay.Reset()