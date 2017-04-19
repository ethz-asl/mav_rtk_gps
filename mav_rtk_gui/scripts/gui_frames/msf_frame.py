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

        # Subplots for position and velocity from MSF.
        self.first_odometry_received = False
        self.first_time_odometry = 0

        self.figure = Figure(figsize=(figureSizeWidth, figureSizeHeight), dpi=75)
        self.figure.subplots_adjust(hspace=.4)

        # Position.
        self.axes_position = self.figure.add_subplot(211)
        self.axes_position.set_xlabel('Time [s]')
        self.axes_position.set_ylabel('Position [m]')
        self.axes_position.grid()

        # Velocity.
        self.axes_velocity = self.figure.add_subplot(212)
        self.axes_velocity.set_xlabel('Time [s]')
        self.axes_velocity.set_ylabel('Velocity [m/s]')
        self.axes_velocity.grid()

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent_window)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(rowspan=4, columnspan=2)

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
        name_space = '/'
        if rospy.has_param('~namespace'):
            name_space = rospy.get_param('~namespace')
            name_space = '/' + name_space + '/'

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

    def odometry_callback(self, msg):
        # Downsample odometry.
        self.odometry_msg_count += 1
        if (self.odometry_msg_count % self.odometry_down_sample_factor) != 0:
            return

        # Data time.
        secs = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / float(1e9)

        if not self.first_odometry_received:
            self.first_odometry_received = True
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

        self.axes_position.set_xlim(min(self.time_odometry), max(self.time_odometry))
        # Add +/- 1 meter to y limit to help visualization.
        self.axes_position.set_ylim(min(min(self.x), min(self.y), min(self.z)) - 1.0,
                                    max(max(self.x), max(self.y), max(self.z)) + 1.0)

        # Update velocity lines.
        self.line_vx.set_xdata(self.time_odometry)
        self.line_vx.set_ydata(self.vx)
        self.line_vy.set_xdata(self.time_odometry)
        self.line_vy.set_ydata(self.vy)
        self.line_vz.set_xdata(self.time_odometry)
        self.line_vz.set_ydata(self.vz)

        self.axes_velocity.set_xlim(min(self.time_odometry), max(self.time_odometry))
        # Add +/- 0.5 meter/s to y limit to help visualization.
        self.axes_velocity.set_ylim(min(min(self.vx), min(self.vy), min(self.vz)) - 0.5,
                                    max(max(self.vx), max(self.vy), max(self.vz)) + 0.5)

        # Final draw.
        self.canvas.draw()
