
import os
import sys
import threading

import rospy
from pr2_guide.msg import *
from pr2_guide.srv import *
from std_msgs.msg import *

class StatusManager

    def __init__( self ):

        # ROS
        ##
        self.nodename = "status_manager"
        rospy.init_node( self.nodename )
        ##
        self.servicename_status_manager_set_status = rospy.getparam( "/pr2_guide/servicename/status_manager/status",
                                                                     "/status_manager/set_status" )
        self.topicname_status_manager_status       = rospy.getparam( "/pr2_guide/topicname/status_manager/status",
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

    def servicehandler_setstatus( self, req ):
        """
        """
        self.setStatus( req.status )
        return StatusManagerSetStatusResponse( True )

    def setStatus( status ):

        self.status = status
        #
        if self.status in self.statushandlers.keys():
            self.statushandlerthread = threading.Thread( target=self.statushandlers[self.status] )
            self.statushandlerthread.start()
            self.statushandlerthread.detach()
        #
        self.publisher_status.publish( self.status )
        
    def statushandler_default( self ):

        setStatus( "waiting" )

    def statushandler_waiting( self ):

    def statushandler_waiting_interaction( self ):

    def statushandler_guiding( self ):

    def statushandler_guiding_halt_waiting( self ):

    def statushandler_guiding_halt_interaction( self ):
