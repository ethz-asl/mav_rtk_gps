#!/usr/bin/env python

#
#  Title:        rtk_info.py
#  Description:  ROS module to create basic GUI to check RTK status.
#

import rospy
from Tkinter import *

from gui_frames.rtk_info_frame import RtkInfoFrame


class BasicGui:
    def __init__(self):
        # Main tkinter window.
        self.root = Tk()
        self.root.wm_title("RTK Info")

        # Screen division.
        self.column_0 = Frame(self.root)
        self.column_0.pack(side = LEFT)

        # Create required frames.
        self.rtk_info_frame = self.create_rtk_info_frame(self.column_0)

        # rospy.spin is not needed in this case.
        self.root.mainloop()

    def create_rtk_info_frame(self, parent_frame):
        rtk_info_frame = RtkInfoFrame(parent_frame)

        return rtk_info_frame

if __name__ == '__main__':
    rospy.init_node('basic_gui')
    rospy.loginfo(rospy.get_name() + ' start')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        basic_gui = BasicGui()
    except rospy.ROSInterruptException:
        pass
