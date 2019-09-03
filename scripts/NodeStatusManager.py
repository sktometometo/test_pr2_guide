#!/usr/bin/env python
# coding: utf-8

import os
import sys
import threading
import time
import signal

import rospy
from pr2_guide.msg import *
from pr2_guide.srv import *
from std_msgs.msg import *

def main():
    # rospy initialization
    rospy.init_node( "status_manager" )
    # initialization of service proxy to demux select of speech_to_text
    servicename_speech_to_text_select = "/demux/select"
    speech_to_text_demux = rospy.ServiceProxy( servicename_speech_to_text_select, DemuxSelect )
    # switch speetch_to_text topic steam to /speech_to_text_app
    speech_to_text_demux( DemuxSelectRequest( "/speech_to_text_app" ) )
    # Initialization of status manager
    sm = StatusManager()
    rate = rospy.Rate(1)
    while sm.isok:
        rate.sleep()
    # switch speetch_to_text topic steam to /speech_to_text_dialogflow_client
    speech_to_text_demux( DemuxSelectRequest( "/speech_to_text_dialogflow_client" ) )
    # rospy shutdown
    rospy.signal_shutdown("finish")
    rospy.spin()

class StatusManager:

    def __init__( self ):

        # ROS
        ##
        self.servicename_status_manager_set_status = rospy.get_param( "/pr2_guide/servicename/status_manager/status",
                                                                     "/status_manager/set_status" )
        self.topicname_status_manager_status       = rospy.get_param( "/pr2_guide/topicname/status_manager/status",
                                                                     "/status_manager/status" )
        ##
        self.service_setstatus = rospy.Service( self.servicename_status_manager_set_status, StatusManagerSetStatus, self.servicehandler_setstatus )
        self.publisher_status = rospy.Publisher( self.topicname_status_manager_status, String, queue_size=10 )

        # signal
        signal.signal( signal.SIGINT, self.sighandler )
        signal.signal( signal.SIGTERM, self.sighandler )
        self.isok = True

        #
        self.status = "default"
        self.statushandlers = {
                                "default":                  self.statushandler_default,
                                "aborting":                 self.statushandler_aborting,
                                "waiting":                  self.statushandler_waiting,
                                "waiting_interaction":      self.statushandler_waiting_interaction,
                                "guiding":                  self.statushandler_guiding,
                                "guiding_halt_waiting":     self.statushandler_guiding_halt_waiting,
                                "guiding_halt_interaction": self.statushandler_guiding_halt_interaction,
                              }
        self.statushandlerthread = None

        #
        time.sleep( 1 ) # 以下の初期化処理時に /status_maanger/status にpublishされるように待つ処理. やばいのできちんと待つ対象を明確にしたい TODO
        ret = self.setStatus( "default" )
        if ret:
            rospy.loginfo( "initialization finished." )
        else:
            rospy.logerr( "initialization has gone wrong." )

    def sighandler( self, signum, frame ):
        self.isok = False

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

    def statushandler_aborting( self ):
        rospy.loginfo( "statushandler_aborting is called." )
        self.isok = False

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
