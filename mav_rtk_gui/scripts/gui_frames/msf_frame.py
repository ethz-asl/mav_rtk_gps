#!/usr/bin/env python

#
#  Title:        msf_frame.py
#  Description:  Reset MSF frame to be attached to a GUI.
#
import rospy
from Tkinter import *
import matplotlib
import helpers

matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import sensor_fusion_comm.srv
from collections import deque

from nav_msgs.msg import Odometry

maxLengthDequeArray = 150
figureSizeWidth = 6.5
figureSizeHeight = 5

class MsfFrame:
    def __init__(self, parent_window):
        # Topics & services Names.
        (self.topic_names, self.service_names) = self.get_topic_service_names()

        # Other parameters.
        self.odometry_down_sample_factor = rospy.get_param('~msf_odometry_down_sample_factor', 10.0)

        self.main_label = Label(parent_window, text='MSF', font="Times 14 bold")
        self.main_label.grid(row=0, columnspan=2)

        # Initialize msf height.
        current_row = 1
        self.init_height_button = Button(parent_window,
                                         text="Init Height",
                                         command=self.init_height_handler)
        self.init_height_button.grid(row=current_row, column=0)
        self.init_height_entry = Entry(parent_window)
        self.init_height_entry.insert(0, '1.0')  # default value
        self.init_height_entry.grid(row=current_row, column=1)

        # Initialize msf scale.
        current_row = 2
        self.init_scale_button = Button(parent_window,
                                        text="Init Scale",
                                        command=self.init_scale_handler)
        self.init_scale_button.grid(row=current_row, column=0)
        self.init_scale_entry = Entry(parent_window)
        self.init_scale_entry.insert(0, '1.0')  # default value
        self.init_scale_entry.grid(row=current_row, column=1)

        # Reset msf view.
        current_row = 3
        self.reset_view_button = Button(parent_window,
                                        text="Reset View",
                                        command=self.reset_view_handler)
        self.reset_view_button.grid(row=current_row, column=0)

        # Local altitude.
        current_row = 4
        self.altitude_label = Label(parent_window, text="Local Altitude [m]: ", font="Sans 8")
        self.altitude_label.grid(row=current_row, column=0)
        self.altitude_status = Label(parent_window, text="", font="Sans 8")
        self.altitude_status.grid(row=current_row, column=1)

        # Subplots for position and velocity from MSF.
        self.figure = Figure(figsize=(figureSizeWidth, figureSizeHeight), dpi=75)
        self.figure.subplots_adjust(hspace=.4)

        self.first_odometry_received = False
        self.reset_plot_view = False
        self.first_time_odometry = 0

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent_window)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(rowspan=4, columnspan=2)

        # Position.
        self.axes_position = []

        # Velocity.
        self.axes_velocity = []

        # Position data.
        self.odometry_msg_count = []
        self.time_odometry = []
        self.x = []
        self.y = []
        self.z = []
        self.line_x = []
        self.line_y = []
        self.line_z = []

        # Velocity data.
        self.vx = []
        self.vy = []
        self.vz = []
        self.line_vx = []
        self.line_vy = []
        self.line_vz = []

        # Init data.
        self.reset_view_handler()

        # Make labels tight
        self.figure.tight_layout()

        # Subscribe to topics.
        rospy.Subscriber(self.topic_names['msf_odometry'], Odometry,
                         self.odometry_callback)

        # Services.
        self.init_height_srv = rospy.ServiceProxy(self.service_names['init_msf_height'],
                                                  sensor_fusion_comm.srv.InitHeight)
        self.init_scale_srv = rospy.ServiceProxy(self.service_names['init_msf_scale'],
                                                 sensor_fusion_comm.srv.InitScale)

    def get_topic_service_names(self):
        topic_names = {}
        service_names = {}

        # Topics.
        topic_names['msf_odometry'] = rospy.get_param('~msf_odometry_topic', 'msf_core/odometry')

        # Services.
        service_names['init_msf_height'] = rospy.get_param('~init_msf_height_srv',
                                                           'pose_sensor_rovio/pose_sensor/initialize_msf_height')
        service_names['init_msf_scale'] = rospy.get_param('~init_msf_scale_srv',
                                                          'pose_sensor_rovio/pose_sensor/initialize_msf_scale')

        # Check if we should add a leading namespace
        name_space = rospy.get_param('~namespace', '')
        for key, value in topic_names.iteritems():
            topic_names[key] = helpers.get_full_namespace(name_space, value)

        for key, value in service_names.iteritems():
            service_names[key] = helpers.get_full_namespace(name_space, value)

        return topic_names, service_names

    def init_scale_handler(self):
        new_scale = float(self.init_scale_entry.get())
        self.init_scale_srv(new_scale)

    def init_height_handler(self):
        new_height = float(self.init_height_entry.get())
        self.init_height_srv(new_height)

    def reset_view_handler(self):

        if not self.first_odometry_received:
            # Init subplots.
            self.axes_position = self.figure.add_subplot(211)
            self.axes_velocity = self.figure.add_subplot(212)

        else:
            # Clear subplots.
            self.axes_position.clear()
            self.axes_velocity.clear()

        # Position.
        self.axes_position.set_xlabel('Time [s]')
        self.axes_position.set_ylabel('Position [m]')
        self.axes_position.grid()

        # Velocity.
        self.axes_velocity.set_xlabel('Time [s]')
        self.axes_velocity.set_ylabel('Velocity [m/s]')
        self.axes_velocity.grid()

        # Position data.
        self.odometry_msg_count = 0
        self.time_odometry = deque([], maxlen=maxLengthDequeArray)
        self.x = deque([], maxlen=maxLengthDequeArray)
        self.y = deque([], maxlen=maxLengthDequeArray)
        self.z = deque([], maxlen=maxLengthDequeArray)
        self.line_x = []
        self.line_y = []
        self.line_z = []

        # Velocity data.
        self.vx = deque([], maxlen=maxLengthDequeArray)
        self.vy = deque([], maxlen=maxLengthDequeArray)
        self.vz = deque([], maxlen=maxLengthDequeArray)
        self.line_vx = []
        self.line_vy = []
        self.line_vz = []

        self.reset_plot_view = True

    def odometry_callback(self, msg):
        # Downsample odometry.
        self.odometry_msg_count += 1
        if (self.odometry_msg_count % self.odometry_down_sample_factor) != 0:
            return

        # Data time.
        secs = msg.header.stamp.to_sec()

        if not self.first_odometry_received or self.reset_plot_view:
            self.first_odometry_received = True
            self.reset_plot_view = False
            self.first_time_odometry = secs

            # Plot empty lines for position.
            self.line_x, self.line_y, self.line_z = self.axes_position.plot([], [], 'r--',
                                                                            [], [], 'g--',
                                                                            [], [], 'b--')
            self.line_x.set_linewidth(2.0)
            self.line_y.set_linewidth(2.0)
            self.line_z.set_linewidth(2.0)

            # Plot empty lines for velocity.
            self.line_vx, self.line_vy, self.line_vz = self.axes_velocity.plot([], [], 'r--',
                                                                               [], [], 'g--',
                                                                               [], [], 'b--')

            self.line_vx.set_linewidth(2.0)
            self.line_vy.set_linewidth(2.0)
            self.line_vz.set_linewidth(2.0)

        # Local altitude.
        self.altitude_status['text'] = str(round(msg.pose.pose.position.z, 2))

        # Position data.
        self.time_odometry.append(secs - self.first_time_odometry)
        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.z.append(msg.pose.pose.position.z)

        # Velocity data.
        self.vx.append(msg.twist.twist.linear.x)
        self.vy.append(msg.twist.twist.linear.y)
        self.vz.append(msg.twist.twist.linear.z)

        # Update position lines.
        self.line_x.set_xdata(self.time_odometry)
        self.line_x.set_ydata(self.x)
        self.line_y.set_xdata(self.time_odometry)
        self.line_y.set_ydata(self.y)
        self.line_z.set_xdata(self.time_odometry)
        self.line_z.set_ydata(self.z)

        # Adjust view based on new data.
        self.axes_position.relim()
        self.axes_position.autoscale_view()

        # Update velocity lines.
        self.line_vx.set_xdata(self.time_odometry)
        self.line_vx.set_ydata(self.vx)
        self.line_vy.set_xdata(self.time_odometry)
        self.line_vy.set_ydata(self.vy)
        self.line_vz.set_xdata(self.time_odometry)
        self.line_vz.set_ydata(self.vz)

        # Adjust view based on new data.
        self.axes_velocity.relim()
        self.axes_velocity.autoscale_view()

        # Final draw.
        self.canvas.draw()
