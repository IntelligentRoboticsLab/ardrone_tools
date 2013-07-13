#!/usr/bin/env python
'''This script starts the Drone Control Interface allowing you to control
drones and apply computer vision software to the camera feed.
'''
import sys
import traceback

import roslib; roslib.load_manifest('ardrone_tools')
import wx

from modules.gui.gui import MainFrame

if __name__ == "__main__":
    app = wx.App()
    frame = MainFrame(app)
    try:
        app.MainLoop()
    except Exception:
        print "\n======== Exception occurred: ========\n"
        traceback.print_exc(file=sys.stdout)
        print "\n=====================================\n"
        frame.Close()
