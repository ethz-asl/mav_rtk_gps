#!/usr/bin/env python

#
#  Title:        rtk_info_frame.py
#  Description:  GPS frame to be attached to a GUI.
#
import rospy
import helpers
from collections import deque

from Tkinter import *

from piksi_rtk_msgs.msg import ReceiverState
from piksi_rtk_msgs.msg import UartState
from piksi_rtk_msgs.msg import BaselineNed
from piksi_rtk_msgs.msg import InfoWifiCorrections
from sensor_msgs.msg import NavSatFix, NavSatStatus

wifiCorrectionsHzAverage = 5  # Compute corrections Hz over wifiCorrectionsHzAverage seconds
altitudeAverageSamples = 10

class RtkInfoFrame:
    def __init__(self, parent_window):
        # Topic Names.
        self.topic_names = self.get_topic_names()

        # Create GUI.
        self.main_label = Label(parent_window, text='RTK Info', font="Times 14 bold")
        self.main_label.grid(row=0, columnspan=2)

        # Type of fix.
        current_row = 1
        self.type_of_fix_label = Label(parent_window, text="Type of fix: ", font="Sans 8")
        self.type_of_fix_label.grid(row=current_row)
        self.type_of_fix_status = Label(parent_window, text="", font="Sans 8")
        self.type_of_fix_status.grid(row=current_row, column=1)

        # Number of satellites.
        current_row = 2
        self.number_sat_label = Label(parent_window, text="Number of satellites: ", font="Sans 8")
        self.number_sat_label.grid(row=current_row)
        self.number_sat_status = Label(parent_window, text="", font="Sans 8")
        self.number_sat_status.grid(row=current_row, column=1)

        # Signal strength.
        current_row = 3
        self.signal_strength_label = Label(parent_window, text="Signal strength: ", font="Sans 8")
        self.signal_strength_label.grid(row=current_row)
        self.signal_strength_status = Label(parent_window, text="", wraplength=300, font="Sans 8")
        self.signal_strength_status.grid(row=current_row, column=1)

        # Uart A throughput.
        current_row = 4
        self.uart_a_throughput_label = Label(parent_window, text="UART A throughput: ", font="Sans 8")
        self.uart_a_throughput_label.grid(row=current_row)
        self.uart_a_throughput_status = Label(parent_window, text="", font="Sans 8")
        self.uart_a_throughput_status.grid(row=current_row, column=1)

        # Uart A crc errors.
        current_row = 5
        self.uart_a_crc_errors_label = Label(parent_window, text="UART A crc errors: ", font="Sans 8")
        self.uart_a_crc_errors_label.grid(row=current_row)
        self.uart_a_crc_errors_status = Label(parent_window, text="", font="Sans 8")
        self.uart_a_crc_errors_status.grid(row=current_row, column=1)

        # Uart B throughput.
        current_row = 6
        self.uart_b_throughput_label = Label(parent_window, text="UART B throughput: ", font="Sans 8")
        self.uart_b_throughput_label.grid(row=current_row)
        self.uart_b_throughput_status = Label(parent_window, text="", font="Sans 8")
        self.uart_b_throughput_status.grid(row=current_row, column=1)

        # Uart B crc errors.
        current_row = 7
        self.uart_b_crc_errors_label = Label(parent_window, text="UART B crc errors: ", font="Sans 8")
        self.uart_b_crc_errors_label.grid(row=current_row)
        self.uart_b_crc_errors_status = Label(parent_window, text="", font="Sans 8")
        self.uart_b_crc_errors_status.grid(row=current_row, column=1)

        # Number of satellites used for RTK fix.
        current_row = 8
        self.number_sat_rtk_label = Label(parent_window, text="Number of satellites used for RTK: ", font="Sans 8")
        self.number_sat_rtk_label.grid(row=current_row)
        self.number_sat_rtk_status = Label(parent_window, text="", font="Sans 8")
        self.number_sat_rtk_status.grid(row=current_row, column=1)

        # Baseline NED.
        current_row = 9
        self.baseline_ned_label = Label(parent_window, text="NED baseline from base station [m]: ", font="Sans 8")
        self.baseline_ned_label.grid(row=current_row)
        self.baseline_ned_status = Label(parent_window, text="", font="Sans 8")
        self.baseline_ned_status.grid(row=current_row, column=1)

        # Navsat Fix altitude.
        current_row = 10
        self.altitude_label = Label(parent_window, text="Navsat fix altitude (avg)[m]: ", font="Sans 8")
        self.altitude_label.grid(row=current_row)
        self.altitude_status = Label(parent_window, text="", font="Sans 8")
        self.altitude_status.grid(row=current_row, column=1)
        self.altitude = deque([], maxlen=altitudeAverageSamples)

        # Number of corrections over wifi.
        current_row = 11
        self.number_corrections_wifi_label = Label(parent_window, text="Number of corrections over wifi: ",
                                                   font="Sans 8")
        self.number_corrections_wifi_label.grid(row=current_row)
        self.number_corrections_wifi_status = Label(parent_window, text="", font="Sans 8")
        self.number_corrections_wifi_status.grid(row=current_row, column=1)

        # Rate corrections over wifi.
        current_row = 12
        self.hz_corrections_wifi_label = Label(parent_window, text="Rate corrections over wifi [Hz]: ", font="Sans 8")
        self.hz_corrections_wifi_label.grid(row=current_row)
        self.hz_corrections_wifi_status = Label(parent_window, text="0", font="Sans 8")
        self.hz_corrections_wifi_status.grid(row=current_row, column=1)
        self.time_first_sample_moving_window = rospy.get_time()
        self.num_corrections_first_sample_moving_window = 0

        # Ping with base station.
        current_row = 13
        self.ping_corrections_wifi_label = Label(parent_window, text="Ping base station [ms]: ", font="Sans 8")
        self.ping_corrections_wifi_label.grid(row=current_row)
        self.ping_corrections_wifi_status = Label(parent_window, text="-1", font="Sans 8")
        self.ping_corrections_wifi_status.grid(row=current_row, column=1)

        # Subscribe to topics.
        rospy.Subscriber(self.topic_names['piksi_receiver_state'], ReceiverState,
                         self.receiver_state_callback)
        rospy.Subscriber(self.topic_names['piksi_uart_state'], UartState,
                         self.uart_state_callback)
        rospy.Subscriber(self.topic_names['piksi_baseline_ned'], BaselineNed,
                         self.baseline_ned_callback)
        rospy.Subscriber(self.topic_names['piksi_wifi_corrections'], InfoWifiCorrections,
                         self.wifi_corrections_callback)
        rospy.Subscriber(self.topic_names['piksi_navsatfix_rtk_fix'], NavSatFix,
                         self.navsatfix_rtk_fix_callback)

    def get_topic_names(self):
        # RTK info topics
        topic_names = {}

        topic_names['piksi_receiver_state'] = rospy.get_param('~piksi_receiver_state_topic',
                                                              'piksi/debug/receiver_state')
        topic_names['piksi_uart_state'] = rospy.get_param('~piksi_uart_state_topic',
                                                          'piksi/debug/uart_state')
        topic_names['piksi_baseline_ned'] = rospy.get_param('~piksi_baseline_ned_topic',
                                                            'piksi/baseline_ned')
        topic_names['piksi_wifi_corrections'] = rospy.get_param('~piksi_num_wifi_corrections_topic',
                                                                'piksi/debug/wifi_corrections')
        topic_names['piksi_navsatfix_rtk_fix'] = rospy.get_param('~piksi_navsatfix_rtk_fix',
                                                                 'piksi/navsatfix_rtk_fix')

        # Check if we should add a leading namespace
        name_space = rospy.get_param('~namespace', '')
        for key, value in topic_names.iteritems():
            topic_names[key] = helpers.get_full_namespace(name_space, value)

        return topic_names

    def receiver_state_callback(self, msg):
        # Type of fix.
        if msg.rtk_mode_fix == True:
            self.type_of_fix_status['bg'] = "green"
            self.type_of_fix_status['fg'] = "white"
            self.type_of_fix_status['text'] = "Fix"
        else:
            self.type_of_fix_status['bg'] = "red"
            self.type_of_fix_status['fg'] = "black"
            self.type_of_fix_status['text'] = "Float"

        # Number of satellites.
        self.number_sat_status['text'] = str(msg.num_sat)

        # Signal strength, use only one decimal digit.
        buffer = '['
        for single_signal_strength in msg.cn0:
            buffer += str(round(single_signal_strength, 1)) + ', '
        # Remove last coma and space.
        buffer = buffer[:-2]
        buffer += ']'

        self.signal_strength_status['text'] = buffer

    def uart_state_callback(self, msg):
        # Uart a throughput.
        self.uart_a_throughput_status['text'] = str(round(msg.uart_a_rx_throughput, 3))

        # Uart a crc errors.
        self.uart_a_crc_errors_status['text'] = str(msg.uart_a_crc_error_count)

        # Uart b throughput.
        self.uart_b_throughput_status['text'] = str(round(msg.uart_b_rx_throughput, 3))

        # Uart b crc errors.
        self.uart_b_crc_errors_status['text'] = str(msg.uart_b_crc_error_count)

    def baseline_ned_callback(self, msg):
        # Number of satellites used for RTK fix.
        self.number_sat_rtk_status['text'] = str(msg.n_sats)

        # Baseline NED, from mm to m.
        buffer = "[%.3f, %.3f, %.3f]" % (msg.n / 1e3, msg.e / 1e3, msg.d / 1e3)
        self.baseline_ned_status['text'] = buffer

    def wifi_corrections_callback(self, msg):
        # Number of corrections received.
        self.number_corrections_wifi_status['text'] = str(msg.received_corrections)

        # Compute rate corrections.
        width_time_window = rospy.get_time() - self.time_first_sample_moving_window

        if width_time_window >= wifiCorrectionsHzAverage:
            samples_in_time_window = msg.received_corrections - self.num_corrections_first_sample_moving_window
            average_corrections_hz = samples_in_time_window / width_time_window
            self.hz_corrections_wifi_status['text'] = str(round(average_corrections_hz, 1))

            # Reset and start again.
            self.num_corrections_first_sample_moving_window = msg.received_corrections
            self.time_first_sample_moving_window = rospy.get_time()

        # Ping base station.
        self.ping_corrections_wifi_status['text'] = str(round(msg.latency, 2))

    def navsatfix_rtk_fix_callback(self, msg):
        self.altitude.append(msg.altitude)
        altitude_avg = sum(self.altitude) / len(self.altitude)
        self.altitude_status['text'] = str(round(altitude_avg, 2))
