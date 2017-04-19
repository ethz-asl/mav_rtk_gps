#!/usr/bin/env python

#
#  Title:        rtk_info.py
#  Description:  ROS module to create a GUI to check RTK status and reset MSF.
#

import rospy
from Tkinter import *

from gui_frames.rtk_info_frame import RtkInfoFrame
from gui_frames.rtk_fix_plot_frame import RtkFixPlotFrame
from gui_frames.msf_frame import MsfFrame

class RtkNpose1:
    def __init__(self):
        # Main tkinter window.
        self.root = Tk()
        self.root.wm_title("RTK & MSF")

        # Screen division.
        self.column_0 = Frame(self.root)
        self.column_0.pack(side = LEFT)
        self.column_1 = Frame(self.root)
        self.column_1.pack(side = LEFT)

        # RTK Stuff.
        self.frame_0_0 = Frame(self.column_0)
        self.frame_0_0.pack(side = TOP)
        self.frame_0_1 = Frame(self.column_0)
        self.frame_0_1.pack(side = TOP)

        # MSF.
        self.frame_1_0 = Frame(self.column_1)
        self.frame_1_0.pack(side = TOP)

        # Create RTK frames.
        self.rtk_info_frame = self.create_rtk_info_frame(self.frame_0_0)
        self.rtk_fix_plot_frame = self.create_rtk_fix_plot_frame(self.frame_0_1)

        # Create MSF frame.
        self.msf_frame = self.create_msf_frame(self.frame_1_0)

        # rospy.spin is not needed in this case.
        self.root.mainloop()

    def create_rtk_info_frame(self, parent_frame):
        rtk_info_frame = RtkInfoFrame(parent_frame)

        return rtk_info_frame

    def create_rtk_fix_plot_frame(self, parent_frame):
        rtk_fix_plot_frame = RtkFixPlotFrame(parent_frame)

        return rtk_fix_plot_frame

    def create_msf_frame(self, parent_frame):
        msf_frame = MsfFrame(parent_frame)

        return msf_frame

if __name__ == '__main__':
    rospy.init_node('rtk_npose1')
    rospy.loginfo(rospy.get_name() + ' start')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rtk_npose1_gui = RtkNpose1()
    except rospy.ROSInterruptException:
        pass
