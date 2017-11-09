#!/usr/bin/env python

#
#  Title:        geodetic_survey.py
#  Description:  Mean a fixed number of GPS points to survey base station position.
#

import rospy
from piksi_rtk_msgs.srv import *
import std_srvs.srv
from sensor_msgs.msg import NavSatFix


class GeodeticSurvey:
    kServiceTimeOutSeconds = 10.0
    kWaitBetweenReadReqAndResSeconds = 0.5

    def __init__(self):
        rospy.loginfo(rospy.get_name() + " start")

        self.latitude_accumulator = 0.0
        self.longitude_accumulator = 0.0
        self.altitude_accumulator = 0.0
        self.number_of_fixes = 0
        self.surveyed_position_set = False

        # Settings
        self.number_of_desired_fixes = rospy.get_param('~number_of_desired_fixes', 5000)
        self.spp_topics_name = rospy.get_param('~spp_topics_name', 'piksi/navsatfix_spp')
        self.write_settings_service_name = rospy.get_param('~write_settings_service_name', 'piksi/settings_write')
        self.save_settings_service_name = rospy.get_param('~save_settings_service_name', 'piksi/settings_save')
        self.read_req_settings_service_name = rospy.get_param('~read_req_settings_service_name',
                                                              'piksi/settings_read_req')
        self.read_resp_settings_service_name = rospy.get_param('~read_resp_settings_service_name',
                                                               'piksi/settings_read_resp')

        # Subscribe.
        rospy.Subscriber(self.spp_topics_name, NavSatFix,
                         self.navsatfix_spp_callback)

        rospy.spin()

    def navsatfix_spp_callback(self, msg):
        self.latitude_accumulator += msg.latitude
        self.longitude_accumulator += msg.longitude
        self.altitude_accumulator += msg.altitude
        self.number_of_fixes += 1

        rospy.loginfo("Received: [%.10f, %.10f, %.1f]; waiting for %d samples" % (
            msg.latitude, msg.longitude, msg.altitude, self.number_of_desired_fixes - self.number_of_fixes))

        if self.number_of_fixes >= self.number_of_desired_fixes and not self.surveyed_position_set:
            lat0 = self.latitude_accumulator / self.number_of_fixes
            lon0 = self.longitude_accumulator / self.number_of_fixes
            alt0 = self.altitude_accumulator / self.number_of_fixes

            if self.set_base_station_position(lat0, lon0, alt0):
                self.surveyed_position_set = True
                rospy.loginfo("Base station position set correctly.")
                rospy.signal_shutdown("Base station position set correctly.")
            else:
                rospy.logerr("Base station position not set correctly.")
                rospy.signal_shutdown("Base station position not set correctly.")

    def set_base_station_position(self, lat0, lon0, alt0):
        everything_ok = []
        rospy.loginfo("Setting Piksi Multi surveyed position to: %.10f, %.10f, %.1f" % (lat0, lon0, alt0))

        # Write settings.
        write_lat0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_lat", "%.10f" % lat0)
        write_lon0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_lon", "%.10f" % lon0)
        write_alt0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_alt", "%.10f" % alt0)

        if write_lat0_ok and write_lon0_ok and write_alt0_ok:
            # Save and check what was actually written to flash
            settings_saved = self.save_settings_to_piksi()
            if settings_saved:
                read_lat0 = self.read_settings_from_piksi("surveyed_position", "surveyed_lat")
                read_lon0 = self.read_settings_from_piksi("surveyed_position", "surveyed_lon")
                read_alt0 = self.read_settings_from_piksi("surveyed_position", "surveyed_alt")

                if read_lat0 and read_lon0 and read_alt0:
                    everything_ok = True
                else:
                    everything_ok = False
            else:
                rospy.logerr("Error while saving base station position to Piksi flash.")
                everything_ok = False
        else:
            rospy.logerr("Error while writing base station position to Piksi.")
            everything_ok = False

        return everything_ok

    def write_settings_to_piksi(self, section_setting, setting, value):
        rospy.wait_for_service(self.write_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        write_settings_service = rospy.ServiceProxy(self.write_settings_service_name, SettingsWrite)
        try:
            rospy.loginfo("Setting %s.%s to %s" % (section_setting, setting, value))
            write_resp = write_settings_service(section_setting=section_setting, setting=setting, value=value)
            return write_resp.success
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def save_settings_to_piksi(self):
        rospy.wait_for_service(self.save_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        save_settings_service = rospy.ServiceProxy(self.save_settings_service_name, std_srvs.srv.SetBool)
        try:
            rospy.loginfo("Saving settings to Piksi Multi flash.")
            save_resp = save_settings_service(True)
            return save_resp.success
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def read_settings_from_piksi(self, section_setting, setting):
        # Read request.
        rospy.wait_for_service(self.read_req_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        read_req_settings_service = rospy.ServiceProxy(self.read_req_settings_service_name, SettingsReadReq)
        try:
            read_req_resp = read_req_settings_service(section_setting=section_setting, setting=setting)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

        if read_req_resp.success:
            # Read req sent, wait before we read the response.
            rospy.sleep(GeodeticSurvey.kWaitBetweenReadReqAndResSeconds)
            # Read response.
            rospy.wait_for_service(self.read_resp_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
            read_resp_settings_service = rospy.ServiceProxy(self.read_resp_settings_service_name, SettingsReadResp)
            try:
                read_resp_resp = read_resp_settings_service()
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))
                return False

            if read_resp_resp.success:
                rospy.loginfo("Read [%s.%s: %s] from Piksi settings." % (
                    read_resp_resp.section_setting, read_resp_resp.setting, read_resp_resp.value))
                return True

        return False


# Main function.
if __name__ == '__main__':
    rospy.init_node('geodetic_survey')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        geodetic_survey = GeodeticSurvey()
    except rospy.ROSInterruptException:
        pass
