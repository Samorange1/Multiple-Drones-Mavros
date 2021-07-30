#!/usr/bin/env python2

# This is the Class for Setup of Multiple Vehicles

from __future__ import division
import unittest
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange


class MultiVehicleSystem:
    def __init__(self,vehicle_no):
        '''Takes in argument: No. of vehicles'''
        #Empty List declarations
        #messages
        self.vehicles = vehicle_no
        self.altitude = []
        self.extended_state = []
        self.global_position = []
        self.imu_data = []
        self.home_position = []
        self.local_position = []
        self.mission_wp = []
        self.state = []
        self.mav_type = []
        
        self.ns = '/uav'

        #Serv
        self.get_param_srv = []
        self.set_arming_srv = []
        self.set_mode_srv = []
        self.wp_clear_srv = []
        self.wp_push_srv = []
        
        #subs
        self.alt_sub = []
        self.ext_state_sub = []
        self.global_pos_sub = []
        self.imu_data_sub = []
        self.home_pos_sub = []
        self.local_pos_sub = []
        self.mission_wp_sub = []
        self.state_sub = []

        for i in range(self.vehicles):
            self.altitude.append(Altitude())
            self.extended_state.append(ExtendedState())
            self.global_position.append(NavSatFix())      
            self.imu_data.append(Imu())
            self.home_position.append(HomePosition())       
            self.local_position.append(PoseStamped())
            self.mission_wp.append(WaypointList())
            self.state.append(State())
            self.mav_type.append(None)       

        self.sub_topics_ready = {}

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        self.service_list = ['/mavros/param/get','/mavros/cmd/arming','/mavros/set_mode',
                            '/mavros/mission/clear','/mavros/mission/push']
        try:
            for j in self.service_list:
                for i in range(self.vehicles):
                    service = self.ns + str(i) + j
                    rospy.wait_for_service(service,service_timeout)         
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")

        for i in range(self.vehicles):
            service = self.ns + str(i) + self.service_list[0]
            self.get_param_srv.append(rospy.ServiceProxy(service, ParamGet))
            
            service = self.ns + str(i) + self.service_list[1]
            self.set_arming_srv.append(rospy.ServiceProxy(service,CommandBool))
            
            service = self.ns + str(i) + self.service_list[2]
            self.set_mode_srv.append(rospy.ServiceProxy(service, SetMode))
            
            service = self.ns + str(i) + self.service_list[3]
            self.wp_clear_srv.append(rospy.ServiceProxy(service,WaypointClear))
            
            service = self.ns + str(i) + self.service_list[4]
            self.wp_push_srv.append(rospy.ServiceProxy(service,WaypointPush))
            
        # ROS subscribers
        self.topics = ['/mavros/altitude','/mavros/extended_state','/mavros/global_position/global',
                    '/mavros/imu/data','/mavros/home_position/home','/mavros/local_position/pose',
                    '/mavros/mission/waypoints','/mavros/state']
        
        for i in range(self.vehicles):
            topic = self.ns + str(i) + self.topics[0]
            self.alt_sub.append(rospy.Subscriber(topic, Altitude,self.altitude_callback,i))
            
            topic = self.ns + str(i) + self.topics[1]
            self.ext_state_sub.append(rospy.Subscriber(topic,ExtendedState,self.extended_state_callback,i))
            
            topic = self.ns + str(i) + self.topics[2]
            self.global_pos_sub.append(rospy.Subscriber(topic,NavSatFix,self.global_position_callback,i))
            
            topic = self.ns + str(i) + self.topics[3]
            self.imu_data_sub.append(rospy.Subscriber(topic,Imu,self.imu_data_callback,i))
            
            topic = self.ns + str(i) + self.topics[4]
            self.home_pos_sub.append(rospy.Subscriber(topic,HomePosition,self.home_position_callback,i))
            
            topic = self.ns + str(i) + self.topics[5]
            self.local_pos_sub.append(rospy.Subscriber(topic,PoseStamped,self.local_position_callback,i))
            
            topic = self.ns + str(i) + self.topics[6]
            self.mission_wp_sub.append(rospy.Subscriber(topic, WaypointList, self.mission_wp_callback,i))
            
            topic = self.ns + str(i) + self.topics[7]
            self.state_sub.append(rospy.Subscriber(topic, State,self.state_callback,i))
    
    #
    # Callback functions
    #
    def altitude_callback(self, data, no):
        self.altitude[no] = data
        s = 'alt' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False
        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready[s] and not math.isnan(data.amsl):
            self.sub_topics_ready[s] = True

    def extended_state_callback(self, data,no):
        s = 'ext_state' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False
        if self.extended_state[no].vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1} for Drone {2}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state[no].vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name, no))

        if self.extended_state[no].landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1} for Drone {2}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state[no].landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name, no))

        self.extended_state[no] = data       
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def global_position_callback(self, data, no):
        s = 'global_pos' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False
        
        self.global_position[no] = data
        
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def imu_data_callback(self, data, no):
        s = 'imu' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False
        
        self.imu_data[no] = data
        
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def home_position_callback(self, data, no):
        s = 'home_pos' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False
        self.home_position[no] = data
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def local_position_callback(self, data, no):
        s = 'local_pos' +str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False

        self.local_position[no] = data
        
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def mission_wp_callback(self, data, no):
        s = 'mission_wp' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False

        if self.mission_wp[no].current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated for Drone {0}: {1}".
                          format(no, data.current_seq))

        self.mission_wp[no] = data
        
        if not self.sub_topics_ready[s]:
            self.sub_topics_ready[s] = True

    def state_callback(self, data, no):
        s = 'state' + str(no)
        if s not in self.sub_topics_ready.keys():
            self.sub_topics_ready[s] = False

        if self.state[no].armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1} for Drone : {2}".format(
                self.state[no].armed, data.armed, no))

        if self.state[no].connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1} for Drone : {2}".format(
                self.state[no].connected, data.connected, no))

        if self.state[no].mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1} for Drone : {2}".format(
                self.state[no].mode, data.mode, no))

        if self.state[no].system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1} for Drone : {2}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state[no].system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name, no))

        self.state[no] = data

        
        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready[s] and data.connected:
            self.sub_topics_ready[s] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout, no):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0} for Drone {1}".format(arm,no))
        old_arm = self.state[no].armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state[no].armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1} for Drone {2}".format(
                    i / loop_freq, timeout, no))
                break
            else:
                try:
                    res = self.set_arming_srv[no](arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command for Drone: {0}".format(no))
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not arm_set:
            raise Exception("failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2} for Drone {3}".
                format(arm, old_arm, timeout, no))

    def set_mode(self, mode, timeout, no):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0} for Drone {1}".format(mode, no))
        old_mode = self.state[no].mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state[no].mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1} for Drone {2}".format(
                    i / loop_freq, timeout, no))
                break
            else:
                try:
                    res = self.set_mode_srv[no](0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command for Drone {0}".format(no))
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not mode_set: 
            raise Exception("failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2} for Drone {3}".
                format(mode, old_mode, timeout, no))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not simulation_ready:
            raise Exception("failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
                format(self.sub_topics_ready, timeout))

    def wait_for_landed_state(self, desired_landed_state, timeout, index, no):
        rospy.loginfo("waiting for landed state for Drone {0}| state: {1}, index: {2}".
                      format(no,mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state[no].landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1} for Drone {2}".
                              format(i / loop_freq, timeout, no))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not landed_state_confirmed:
            raise Exception("landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3} for Drone {4}".
                            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                            desired_landed_state].name, mavutil.mavlink.enums[
                            'MAV_LANDED_STATE'][self.extended_state[no].landed_state].name,
                            index, timeout, no))

    def wait_for_vtol_state(self, transition, timeout, index, no):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1} for Drone {2}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index, no))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state[no].vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1} for Drone {2}".format(
                    i / loop_freq, timeout, no))
                transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not transitioned:
            raise Exception("transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3} for Drone {4}".
                format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                    mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    self.extended_state[no].vtol_state].name, index, timeout, no))

    def clear_wps(self, timeout, no):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp[no].waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1} for Drone {2}".
                              format(i / loop_freq, timeout, no))
                break
            else:
                try:
                    res = self.wp_clear_srv[no]()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command for Drone {0}".format(no))
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not wps_cleared:
            raise Exception("failed to clear waypoints | timeout(seconds): {0} for Drone {1}".format(timeout,no))

    def send_wps(self, waypoints, timeout, no):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints for Drone {0}".format(no))
        if self.mission_wp[no].waypoints:
            rospy.loginfo("FCU already has mission waypoints for Drone {0}".format(no))

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv[no](start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred for Drone {0}".format(no))
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp[no].waypoints):
                    rospy.loginfo("number of waypoints transferred for Drone {0}: {1}".
                                  format(no, len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success  for Drone {0}| seconds: {1} of {2}".
                              format(no, i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not (wps_sent and wps_verified):
            raise Exception ("mission could not be transferred and verified for Drone {0}| timeout(seconds): {1}".format(no, timeout))

    def wait_for_mav_type(self, timeout, no):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv[no]('MAV_TYPE')
                if res.success:
                    self.mav_type[no] = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type[no]]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not res.success:
            raise Exception("MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout))

    