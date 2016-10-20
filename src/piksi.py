#!/usr/bin/env python

#
#  Title:        piksi.py
#  Description:  ROS Driver for the Piksi RTK GPS module
#  Dependencies: piksi_tool (https://github.com/swift-nav/piksi_tools), tested with v1.2.1
#

import rospy
import sys

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from piksi_rtk_gps.msg import *

# Import Piksi SBP library
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import *
from sbp.logging import *
from sbp.system import *
from sbp.tracking import * # WARNING: tracking is part of the draft messages, could be removed in future releases of libsbp
from sbp.piksi import * # WARNING: piksi is part of the draft messages, could be removed in future releases of libsbp
import sbp.version

import time

def make_callback(sbp_type, ros_message, pub, attrs):
    """
    Dynamic generator for callback functions for message types from
    the SBP library. 
    Inputs: 'sbp_type' name of SBP message type
            'ros_message' ROS message type with SBP format
            'pub' ROS publisher for ros_message
            'attrs' array of attributes in SBP/ROS message
    Returns: callback function 'callback'
    """
    def callback(msg, **metadata):
        sbp_message = sbp_type(msg)
        for attr in attrs:
            setattr(ros_message, attr, getattr(sbp_message, attr))
        pub.publish(ros_message)
    return callback

def init_callback_and_publisher(topic_name, ros_datatype, sbp_msg_type, callback_data_type, *attrs):
    """
    Initializes the callback function and ROS publisher for an SBP
    message type.
    Inputs: 'topic_name' name of ROS topic for publisher
            'ros_datatype' ROS custom message type
            'sbp_msg_type' name of SBP message type for callback function
            'callback_data_type' name of SBP message type for SBP library
            '*attrs' array of attributes in ROS/SBP message
    """
    if not rospy.has_param('~publish_' + topic_name):
        rospy.set_param('~publish_' + topic_name, False)
    if rospy.get_param('~publish_' + topic_name):
        pub = rospy.Publisher(rospy.get_name() + '/' + topic_name, ros_datatype, queue_size = 10)
        ros_message = ros_datatype()
        
        # Add callback function
        callback_function = make_callback(callback_data_type, ros_message, pub, attrs)
        handler.add_callback(callback_function, msg_type=sbp_msg_type)

 
def navsatfix_callback(msg_raw, **metadata):
    """
    Callback function for SBP_MSG_POS_LLH message types. Publishes
    NavSatFix messages.
    """
    msg = MsgPosLLH(msg_raw)
    #print metadata

    navsatfix_msg.header.stamp = rospy.Time.now()
    navsatfix_msg.latitude = msg.lat
    navsatfix_msg.longitude = msg.lon
    navsatfix_msg.altitude = msg.height
    
    # SPP GPS messages
    if msg.flags == 0 and publish_spp:
        navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
        navsatfix_msg.position_covariance = [var_spp_x, 0, 0,
                                             0, var_spp_y, 0,
                                             0, 0, var_spp_z]
        pub_spp.publish(navsatfix_msg)
        
    # RTK GPS messages
    elif (msg.flags == 1 or msg.flags == 2) and publish_rtk:
        
        navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX


        if msg.flags == 2: #RTK float
            navsatfix_msg.position_covariance = [var_rtk_float_x, 0, 0,
                                                 0, var_rtk_float_y, 0,
                                                 0, 0, var_rtk_float_z]
            pub_rtk_float.publish(navsatfix_msg)
        else: #RTK fix
            navsatfix_msg.position_covariance = [var_rtk_fix_x, 0, 0,
                                                 0, var_rtk_fix_y, 0,
                                                 0, 0, var_rtk_fix_z]
            pub_rtk_fix.publish(navsatfix_msg)


        #Update debug msg and publish
        debug_msg.rtk_mode_fix = True if (msg.flags == 1) else False
        publish_piksidebug_msg()


def baseline_callback(msg_raw, **metadata):
    """
    Callback function for SBP_MSG_BASELINE_NED message types.
    Publishes PiksiBaseline messages.
    """
    msg = MsgBaselineNED(msg_raw)

    baseline_msg.header.stamp = rospy.Time.now()
    baseline_msg.baseline.x = msg.n
    baseline_msg.baseline.y = msg.e
    baseline_msg.baseline.z = msg.d
    baseline_msg.mode_fixed = msg.flags

    pub_piksibaseline.publish(baseline_msg)

def heartbeat_callback(msg_raw, **metadata):
    """
    Callback function for SBP_MSG_HEARTBEAT message types.
    Publishes msg_heartbeat messages.
    """
    msg = MsgHeartbeat(msg_raw)

    heartbeat_msg.system_error = msg.flags & 0x01
    heartbeat_msg.io_error = msg.flags & 0x02
    heartbeat_msg.swift_nap_error = msg.flags & 0x04
    heartbeat_msg.sbp_minor_version = (msg.flags & 0xFF00) >> 8
    heartbeat_msg.sbp_major_version = (msg.flags & 0xFF0000) >> 16
    heartbeat_msg.external_antenna_present = (msg.flags & 0x80000000) >> 31

    pub_heartbeat.publish(heartbeat_msg)

    #Update debug msg and publish
    debug_msg.system_error = heartbeat_msg.system_error
    debug_msg.io_error = heartbeat_msg.io_error
    debug_msg.swift_nap_error = heartbeat_msg.swift_nap_error
    debug_msg.external_antenna_present = heartbeat_msg.external_antenna_present
    publish_piksidebug_msg()

def tracking_state_callback(msg_raw, **metadata):
    """
    Callback function for SBP_MSG_TRACKING_STATE message types.
    Publishes msg_tracking_state messages.
    """
    msg = MsgTrackingState(msg_raw)

    tracking_state_msg.state = []
    tracking_state_msg.sat = []
    tracking_state_msg.code = []
    tracking_state_msg.cn0 = []

    for single_tracking_state in msg.states:
        #take only running tracking
        track_running = single_tracking_state.state & 0x01
        if track_running:
            tracking_state_msg.state.append(single_tracking_state.state)
            tracking_state_msg.sat.append(single_tracking_state.sid.sat)
            tracking_state_msg.code.append(single_tracking_state.sid.code)
            tracking_state_msg.cn0.append(single_tracking_state.cn0)

    #publish if there's at least one element in each array
    if len(tracking_state_msg.state) and len(tracking_state_msg.sat) and len(tracking_state_msg.code) and len(tracking_state_msg.cn0):
        pub_tracking_state.publish(tracking_state_msg)

        #Update debug msg and publish
        debug_msg.num_sat = 0 #count number of satellites used to track
        for tracking_running in tracking_state_msg.state:
            debug_msg.num_sat += tracking_running

        debug_msg.sat = tracking_state_msg.sat
        debug_msg.cn0 = tracking_state_msg.cn0
        debug_msg.tracking_running = tracking_state_msg.state
        publish_piksidebug_msg()



def publish_piksidebug_msg():
    """
    Callback function to publish PiksiDebug msg.
    """
    if publish_piksidebug:
        pub_piksidebug.publish(debug_msg)

def uart_state_callback(msg_raw, **metadata):
    """
    Callback function for SBP_MSG_UART_STATE message types.
    Publishes msg_uart_state messages.
    """
    msg = MsgUartStateDepa(msg_raw)

    uart_state_msg = msg_uart_state()

    uart_state_msg.uart_a_tx_throughput = msg.uart_a.tx_throughput
    uart_state_msg.uart_a_rx_throughput = msg.uart_a.rx_throughput
    uart_state_msg.uart_a_crc_error_count = msg.uart_a.crc_error_count
    uart_state_msg.uart_a_io_error_count = msg.uart_a.io_error_count
    uart_state_msg.uart_a_tx_buffer_level = msg.uart_a.tx_buffer_level
    uart_state_msg.uart_a_rx_buffer_level = msg.uart_a.rx_buffer_level

    uart_state_msg.uart_b_tx_throughput = msg.uart_b.tx_throughput
    uart_state_msg.uart_b_rx_throughput = msg.uart_b.rx_throughput
    uart_state_msg.uart_b_crc_error_count = msg.uart_b.crc_error_count
    uart_state_msg.uart_b_io_error_count = msg.uart_b.io_error_count
    uart_state_msg.uart_b_tx_buffer_level = msg.uart_b.tx_buffer_level
    uart_state_msg.uart_b_rx_buffer_level = msg.uart_b.rx_buffer_level

    uart_state_msg.latency_avg = msg.latency.avg
    uart_state_msg.latency_lmin = msg.latency.lmin
    uart_state_msg.latency_lmax = msg.latency.lmax
    uart_state_msg.latency_current = msg.latency.current

    pub_piksi_uart_state.publish(uart_state_msg)


# Main function.    
if __name__ == '__main__':   
    rospy.init_node('piksi')

    #Print info
    rospy.sleep(0.5) #wait for a while for init to complete before printing
    rospy.loginfo("init_node")
    rospy.loginfo("libsbp version currently used: " + sbp.version.get_git_version())

    # Open a connection to Piksi using the default baud rate (1Mbaud)
    serial_port = rospy.get_param('~serial_port')

    try:
        driver = PySerialDriver(serial_port, baud=1000000)
    except SystemExit:
        rospy.logerr("Piksi not found on serial port '%s'", serial_port)
    
    # Create a handler to connect Piksi driver to callbacks
    handler = Handler(Framer(driver.read, driver.write, verbose=True))

    # Read settings
    var_spp_x = rospy.get_param('~var_spp_x')
    var_spp_y = rospy.get_param('~var_spp_y')
    var_spp_z = rospy.get_param('~var_spp_z')
    var_rtk_float_x = rospy.get_param('~var_rtk_float_x')
    var_rtk_float_y = rospy.get_param('~var_rtk_float_y')
    var_rtk_float_z = rospy.get_param('~var_rtk_float_z')
    var_rtk_fix_x = rospy.get_param('~var_rtk_fix_x')
    var_rtk_fix_y = rospy.get_param('~var_rtk_fix_y')
    var_rtk_fix_z = rospy.get_param('~var_rtk_fix_z')

    # Navigation settings
    if not rospy.has_param('~publish_navsatfix_rtk'):
        rospy.set_param('~publish_navsatfix_rtk', False)
    if not rospy.has_param('~publish_navsatfix_spp'):
        rospy.set_param('~publish_navsatfix_spp', False)
    if not rospy.has_param('~publish_piksibaseline'):
        rospy.set_param('~publish_piksibaseline', False)

    publish_spp = rospy.get_param('~publish_navsatfix_spp')
    publish_rtk = rospy.get_param('~publish_navsatfix_rtk')
    publish_piksibaseline = rospy.get_param('~publish_piksibaseline')

    # Logging settings
    if not rospy.has_param('~publish_log'):
        rospy.set_param('~publish_log', False)
    publish_log = rospy.get_param('~publish_log')

    # System settings
    if not rospy.has_param('~publish_heartbeat'):
        rospy.set_param('~publish_heartbeat', False)
    publish_heartbeat = rospy.get_param('~publish_heartbeat')

    # Tracking settings
    if not rospy.has_param('~publish_tracking_state'):
        rospy.set_param('~publish_tracking_state', False)
    publish_tracking_state = rospy.get_param('~publish_tracking_state')

    # Debug settings
    if not rospy.has_param('~publish_piksidebug'):
        rospy.set_param('~publish_piksidebug', False)
    publish_piksidebug = rospy.get_param('~publish_piksidebug')

    if not rospy.has_param('~publish_uart_state'):
        rospy.set_param('~publish_uart_state', False)
    publish_uart_state = rospy.get_param('~publish_uart_state')


    # Generate publisher and callback function for navsatfix messages
    if publish_spp or publish_rtk:
        if publish_rtk:
            pub_rtk_float = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_float', NavSatFix, queue_size = 10)
            pub_rtk_fix = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_fix', NavSatFix, queue_size = 10)
        if publish_spp:
            pub_spp = rospy.Publisher(rospy.get_name() + '/navsatfix_spp', NavSatFix, queue_size = 10)
            
        # Define fixed attributes of the NavSatFixed message
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = rospy.get_param('~frame_id')
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
        
        handler.add_callback(navsatfix_callback, msg_type=SBP_MSG_POS_LLH)
    
    # Generate publisher and callback function for PiksiBaseline messages
    if publish_piksibaseline:
        pub_piksibaseline = rospy.Publisher(rospy.get_name() + '/piksibaseline', PiksiBaseline, queue_size = 10)
        baseline_msg = PiksiBaseline()
        handler.add_callback(baseline_callback, msg_type=SBP_MSG_BASELINE_NED)

    
    # Initialize Navigation messages
    init_callback_and_publisher('baseline_ecef', msg_baseline_ecef, SBP_MSG_BASELINE_ECEF, MsgBaselineECEF,  
                                'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
    init_callback_and_publisher('baseline_ned', msg_baseline_ned, SBP_MSG_BASELINE_NED, MsgBaselineNED,  
                                'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
    init_callback_and_publisher('dops', msg_dops, SBP_MSG_DOPS, MsgDops, 'tow', 'gdop', 'pdop', 'tdop', 'hdop', 'vdop')
    init_callback_and_publisher('gps_time', msg_gps_time, SBP_MSG_GPS_TIME, MsgGPSTime, 'wn', 'tow', 'ns', 'flags')
    init_callback_and_publisher('pos_ecef', msg_pos_ecef, SBP_MSG_POS_ECEF, MsgPosECEF, 
                                'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
    init_callback_and_publisher('pos_llh', msg_pos_llh, SBP_MSG_POS_LLH, MsgPosLLH, 
                                'tow', 'lat', 'lon', 'height', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')

    init_callback_and_publisher('vel_ecef', msg_vel_ecef, SBP_MSG_VEL_ECEF, MsgVelECEF, 
                                'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
    init_callback_and_publisher('vel_ned', msg_vel_ned, SBP_MSG_VEL_NED, MsgVelNED, 
                                'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')

    # Initialize Logging messages
    init_callback_and_publisher('log', msg_log, SBP_MSG_LOG, MsgLog, 'level', 'text')

    # Generate publisher and callback function for System messages
    if publish_heartbeat:
        pub_heartbeat = rospy.Publisher(rospy.get_name() + '/heartbeat', msg_heartbeat, queue_size = 10)
        heartbeat_msg = msg_heartbeat()
        handler.add_callback(heartbeat_callback, msg_type=SBP_MSG_HEARTBEAT)

    # Generate publisher and callback function for Tracking messages
    if publish_tracking_state:
        pub_tracking_state = rospy.Publisher(rospy.get_name() + '/tracking_state', msg_tracking_state, queue_size = 10)
        tracking_state_msg = msg_tracking_state()
        handler.add_callback(tracking_state_callback, msg_type=SBP_MSG_TRACKING_STATE)

    # Generate publisher and callback function for Debug messages
    #init debug msg, required even tough publish_piksidebug is false to avoid run time errors
    debug_msg = PiksiDebug()
    debug_msg.num_sat = 0 #Unkown
    debug_msg.rtk_mode_fix = False #Unkown
    debug_msg.sat = [] #Unkown
    debug_msg.cn0 = [] #Unkown
    debug_msg.tracking_running = [] #Unkown
    debug_msg.system_error = 255 #Unkown
    debug_msg.io_error = 255 #Unkown
    debug_msg.swift_nap_error = 255 #Unkown
    debug_msg.external_antenna_present = 255 #Unkown
    if publish_piksidebug:
        pub_piksidebug = rospy.Publisher(rospy.get_name() + '/debug', PiksiDebug, queue_size = 10)


    #uart state
    if publish_uart_state:
        handler.add_callback(uart_state_callback, msg_type=SBP_MSG_UART_STATE_DEPA)
        pub_piksi_uart_state = rospy.Publisher(rospy.get_name() + '/uart_state', msg_uart_state, queue_size = 10)

    
    handler.start()
    
    # Keep script alive
    while not rospy.is_shutdown():
        rospy.sleep(0.5)