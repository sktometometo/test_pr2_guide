
import os
import sys
import threading

import rospy
from pr2_guide.msg import *
from pr2_guide.srv import *
from std_msgs.msg import *

class StatusManager

    def __init__( self ):

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
        
        # ROS
        rospy.init_node( "StatusManager" )
        self.rosservice_setstatus = rospy.Service( "/StatusManager/Service/set_status", StatusManagerService_SetStatus, self.servicehandler_setstatus )
        self.rospublisher_setstatus = rospy.Publisher( "/StatusManager/Status", String, queue_size=10 )

    def servicehandler_setstatus( self, req ):
        """
        """
        self.setStatus( req.status )
        return StatusManagerService_SetStatusResponse(True)

    def setStatus( status ):

        self.status = status
        #
        if self.status in self.statushandlers.keys():
            self.statushandlerthread = threading.Thread( target=self.statushandlers[self.status] )
            self.statushandlerthread.start()
            self.statushandlerthread.detach()
        #
        self.rospublisher_setstatus.publish( self.status )
        
    def statushandler_default( self ):

        setStatus( "waiting" )

    def statushandler_waiting( self ):

    def statushandler_waiting_interaction( self ):

    def statushandler_guiding( self ):

    def statushandler_guiding_halt_waiting( self ):

    def statushandler_guiding_halt_interaction( self ):
