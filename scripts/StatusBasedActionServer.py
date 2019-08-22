#!/usr/bin/env python
# coding: utf-8

import yaml
import sys
import os
import ros
from pr2_guide.srv import *

class StatusBasedAction:

    def __init__( self ):
        """
        """

        # status and actions
        self.status_and_action = {
                      "waiting" :                  self.actionhandler_waiting ,
                      "waiting_interaction" :      self.actionhandler_waiting_interaction ,
                      "guiding" :                  self.actionhandler_guiding ,
                      "guiding_halt_waiting" :     self.actionhandler_guiding_halt_waiting ,
                      "guiding_halt_interaction" : self.actionhandler_guiding_halt_interaction ,
                      "default" :                  self.actionhandler_default 
                    }
        self.status = "default"

        # ROS
        rospy.init_node( "/pr2_guide/status_based_action_node" )
        self.serv_set_action_status = rospy.Service( "/pr2_guide/status_based_action_server/set_action_status", StatusBasedActionServer_SetStatus, self.handler_setActionStatus )
        self.serv_get_action_status = rospy.Service( "/pr2_guide/status_based_action_server/get_action_status", StatusBasedActionServer_GetStatus, self.handler_getActionStatus )

    def handler_setActionStatus( self, req ):
        
        if self.setActionStatus( req.status ):
            return StatusBasedActionServer_SetStatusResponse( True )
        else:
            return StatusBasedActionServer_SetStatusResponse( False )

    def handler_getActionStatus( self, req ):

        return StatusBasedActionServer_GetStatusResponse( self.getActionStatus )

    def setActionStatus( self, status ):

        if status in self.status_and_action.keys():
            self.status = status
            return True
        else:
            return False

    def getActionStatus( self ):

        return self.status

    def setMotionStatus( self, status ):

        rospy.wait_for_service( "/pr2_guide/status_based_motion_server/set_motion_status" )
        try:
            set_motion_status = rospy.ServiceProxy( "/pr2_guide/status_based_motion_server/set_motion_status" )
            ret = set_motion_status( status )
            return ret.ret
        except rospy.ServiceException, e:
            # echo
            return False

    def getMotionStatus( self ):

        rospy.wait_for_service( "/pr2_guide/status_based_motion_server/get_motion_status" )
        try:
            get_motion_status = rospy.ServiceProxy( "/pr2_guide/status_based_motion_server/get_motion_status" )
            ret = get_motion_status( )
            return ret.status
        except rospy.ServiceException, e:
            # echo
            return None

    def spin( self ):

        while True:
            self.status_and_action[ self.status ]()

    def actionhandler_default( self ):

        if self.getMotionStatus() != "default":
            self.setMotionStatus( "default" )

        self.setActionStatus( "waiting" )
        return
        
    def actionhandler_waiting( self ):
        """
        """
        if self.getMotionStatus() != "waiting":
            self.setMotionStatus( "waiting" )

        return

    def actionhandler_waiting_interaction( self ):

        if self.getMotionStatus() != "waiting_interaction":
            self.setMotionStatus( "waiting_interaction" )

        ret = True

        if ret:
            self.setActionStatus( "waiting_interaction" )
        else:
            self.setActionStatus( "waiting" )

    def actionhandler_guiding( self ):

        if self.getMotionStatus() != "guiding":
            self.setMotionStatus( "guiding" )


