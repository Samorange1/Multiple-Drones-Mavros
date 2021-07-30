#!/usr/bin/env python2
from __future__ import division
from pickle import NONE

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from MultiVehicleSetup2 import MultiVehicleSystem
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler


class MultiVehicleTest(MultiVehicleSystem, object):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def __init__(self):
        self.vehicle_no = int(input("Enter number of vehicles to use: "))
        
        super(MultiVehicleTest, self).__init__(self.vehicle_no)
        self.pos = []
        self.radius = 1
        self.pos_setpoint_pub = []
        self.drone_pos_thread = []
        self.landed_state_thread = []
        for i in range(self.vehicle_no):
            self.pos.append(PoseStamped())
            topic = self.ns + str(i) + '/mavros/setpoint_position/local'
            self.pos_setpoint_pub.append(rospy.Publisher(topic, PoseStamped, queue_size=1))
            self.drone_pos_thread.append(None)
            self.landed_state_thread.append(None)
        # send setpoints in seperate thread to better prevent failsafe
        
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
        
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        for i in range(3):
            self.pos[i].header = Header()
            self.pos[i].header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            for i in range(self.vehicle_no):
                self.pos[i].header.stamp = rospy.Time.now()
                self.pos_setpoint_pub[i].publish(self.pos[i])
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset,no):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position[no].pose.position.x, self.local_position[no].pose.
                position.y, self.local_position[no].pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position[no].pose.position.x,
                        self.local_position[no].pose.position.y,
                        self.local_position[no].pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout, no, yaw):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos[no].pose.position.x = x
        self.pos[no].pose.position.y = y
        self.pos[no].pose.position.z = z
        rospy.loginfo(
                "attempting to reach position for DRONE {0} | x: {1}, y: {2}, z: {3} | current position x: {4:.2f}, y: {5:.2f}, z: {6:.2f}".
                format(no,x, y, z, self.local_position[no].pose.position.x,
                    self.local_position[no].pose.position.y,
                    self.local_position[no].pose.position.z))   
        
         
        yaw_rad = math.radians(yaw) #degree
        quaternion = quaternion_from_euler(0, 0, yaw_rad)
        self.pos[no].pose.orientation = Quaternion(*quaternion)
        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos[no].pose.position.x,
                                    self.pos[no].pose.position.y,
                                    self.pos[no].pose.position.z, self.radius,no):
                    rospy.loginfo("position reached DRONE {0} | seconds: {1} of {2}".format(
                        no, i / loop_freq, timeout))
                    reached = True
                    break            
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if not reached:
            raise Exception("took too long to get to position for Drone {0} | current position x: {1:.2f}, y: {2:.2f}, z: {3:.2f} | timeout(seconds): {4}".
                    format(no, self.local_position[no].pose.position.x,
                        self.local_position[no].pose.position.y,
                        self.local_position[no].pose.position.z, timeout))
    
   

    def landing(self,land_state,t1,i1,no,arm_state,t2):
        self.wait_for_landed_state(land_state,t1,i1,no)
        self.set_arm(arm_state,t2,no)


    #
    # Test method
    #
    def testMultiVehicle(self):
        """Test offboard position control"""
        
        # make sure the simulation is ready to start the mission
        # self.wait_for_topics(60)
        # for i in range(self.vehicle_no):
        #     self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        #                                 10, -1,i)
        #     self.set_mode("OFFBOARD", 5, i)
        #     self.set_arm(True, 5, i)
        #     rospy.sleep(0.5)

        # rospy.loginfo("run mission")
        # positions = ((0, 0, 0, 0), (0, 0, 5,0), (20, 0, 5,0), (20, 20, 5,90), (0, 20, 5,180), (0, 0, 5, 270), (0, 0, 5, 0))

        # for i in xrange(len(positions)):
        #     for j in range(self.vehicle_no):
        #         self.drone_pos_thread[j] = Thread(target= self.reach_position, args=(positions[i][0], positions[i][1],positions[i][2], 30, j, positions[i][3]))
        #         self.drone_pos_thread[j].start()
        #         rospy.sleep(1)
            
        # for i in range(self.vehicle_no):
        #     self.set_mode("AUTO.LAND", 5, i)
        #     self.landed_state_thread[i] = Thread(target = self.landing , 
        #                             args =(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        #                             45, 0, i, False, 5))
        #     self.landed_state_thread[i].start()
        for i in range(self.vehicle_no):
            self.set_arm(True, 5, i)



if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    mav = MultiVehicleTest()
    mav.testMultiVehicle()
