import rospy
from Tkinter import *
import matplotlib
import helpers

matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from collections import deque
from piksi_rtk_msgs.msg import ReceiverState

maxLengthDequeArray = 100
figureSizeWidth = 5.5
figureSizeHeight = 2


class RtkFixPlotFrame:
    def __init__(self, parent_window):
        # Topic Names.
        self.topic_names = self.get_topic_names()

        self.main_label = Label(parent_window, text="RTK Fix Plot", font="Times 14 bold")
        self.main_label.grid(row=0, columnspan=2)

        # Plot for RTK fix.
        self.first_receiver_state_received = False
        self.first_time_receiver_state = 0

        self.figure = Figure(figsize=(figureSizeWidth, figureSizeHeight), dpi=75)

        self.axes_rtk_fix = self.figure.add_subplot(111)
        self.axes_rtk_fix.set_xlabel('Time [s]')
        self.axes_rtk_fix.set_ylabel('RTK Fix')
        self.axes_rtk_fix.grid()

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent_window)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(rowspan=4, columnspan=2)

        # Position data.
        self.odometry_msg_count = 0
        self.time_rtk_fix = deque([], maxlen=maxLengthDequeArray)
        self.rtk_fix = deque([], maxlen=maxLengthDequeArray)
        self.line_rtk_fix = []

        # Subscribe to topics.
        rospy.Subscriber(self.topic_names['piksi_receiver_state'], ReceiverState,
                         self.receiver_state_callback)

    def get_topic_names(self):
        # RTK info topics.
        topic_names = {}

        topic_names['piksi_receiver_state'] = rospy.get_param('~piksi_receiver_state_topic',
                                                              'piksi/debug/receiver_state')

        # Check if we should add a leading namespace
        name_space = '/'
        if rospy.has_param('~namespace'):
            name_space = rospy.get_param('~namespace')
            name_space = '/' + name_space + '/'

        for key, value in topic_names.iteritems():
            topic_names[key] = helpers.get_full_namespace(name_space, value)

        return topic_names

    def receiver_state_callback(self, msg):
        # Data time.
        secs = rospy.get_time()

        if not self.first_receiver_state_received:
            self.first_receiver_state_received = True
            self.first_time_receiver_state = secs

            # Plot empty line.
            self.line_rtk_fix, = self.axes_rtk_fix.plot([], [], 'k--*')
            self.line_rtk_fix.set_linewidth(2.0)

        # Data.
        self.time_rtk_fix.append(secs - self.first_time_receiver_state)
        if msg.rtk_mode_fix == True:
            self.rtk_fix.append(1.0)
        else:
            self.rtk_fix.append(0.0)

        # Update plot.
        self.line_rtk_fix.set_xdata(self.time_rtk_fix)
        self.line_rtk_fix.set_ydata(self.rtk_fix)

        self.axes_rtk_fix.set_xlim(min(self.time_rtk_fix), max(self.time_rtk_fix))
        # Help line visualization.
        self.axes_rtk_fix.set_ylim(-0.1,
                                   1.1)

        # Final draw.
        self.canvas.draw()
