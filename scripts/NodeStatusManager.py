#!/usr/bin/env python
# coding: utf-8

import os
import sys
import threading

import rospy
from pr2_guide.msg import *
from pr2_guide.srv import *
from std_msgs.msg import *

def main():
    sm = StatusManager()
    rospy.spin()

class StatusManager:

    def __init__( self ):

        # ROS
        ##
        self.nodename = "status_manager"
        rospy.init_node( self.nodename )
        ##
        self.servicename_status_manager_set_status = rospy.get_param( "/pr2_guide/servicename/status_manager/status",
                                                                     "/status_manager/set_status" )
        self.topicname_status_manager_status       = rospy.get_param( "/pr2_guide/topicname/status_manager/status",
                                                                     "/status_manager/status" )
        ##
        self.service_setstatus = rospy.Service( self.servicename_status_manager_set_status, StatusManagerSetStatus, self.servicehandler_setstatus )
        self.publisher_status = rospy.Publisher( self.topicname_status_manager_status, String, queue_size=10 )

        #
        self.status = "default"
        self.statushandlers = {
                                "default":                  self.statushandler_default,
                                "waiting":                  self.statushandler_waiting,
                                "waiting_interaction":      self.statushandler_waiting_interaction,
                                "guiding":                  self.statushandler_guiding,
                                "guiding_halt_waiting":     self.statushandler_guiding_halt_waiting,
                                "guiding_halt_interaction": self.statushandler_guiding_halt_interaction,
                              }
        self.statushandlerthread = None

        #
        if self.setStatus( "default" ):
            rospy.loginfo( "initialization finished." )
        else:
            rospy.logerr( "initialization has gone wrong." )

    def servicehandler_setstatus( self, req ):
        """
        """
        ret = self.setStatus( req.status )
        return StatusManagerSetStatusResponse( ret )

    def setStatus( self, status ):

        #
        if status in self.statushandlers.keys():
            self.status = status
            self.statushandlerthread = threading.Thread( target=self.statushandlers[self.status] )
            self.statushandlerthread.start()
            self.publisher_status.publish( self.status )
            return True
        else:
            return False
        
    def statushandler_default( self ):
        rospy.loginfo( "statushandler_default is called." )
        self.setStatus( "waiting" )

    def statushandler_waiting( self ):
        rospy.loginfo( "statushandler_waiting is called." )

    def statushandler_waiting_interaction( self ):
        rospy.loginfo( "statushandler_waiting_interaction is called." )

    def statushandler_guiding( self ):
        rospy.loginfo( "statushandler_guiding is called." )

    def statushandler_guiding_halt_waiting( self ):
        rospy.loginfo( "statushandler_guiding_halt_waiting is called." )

    def statushandler_guiding_halt_interaction( self ):
        rospy.loginfo( "statushandler_guiding_halt_interaction is called." )

if __name__ == "__main__":
    main()
