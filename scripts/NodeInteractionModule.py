#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle

import rospy
from pr2_guide.srv import *
from pr2_guide.msg import *

import dialogflow_v2 as dialogflow

import threading

class InteractionModule

    def __init__( self, 
                  dialogflow_project_id, 
                  dialogflow_session_id,
                  language_code = "ja" ):
        
        # all member methods
        self.language_code = ""
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
        self.isok = False
        self.sttbuffer = []

        # dialog flow
        self.df_sessionclient = dialogflow.SessionClient()
        self.df_session       = self.df_sessionclient.session_path( dialogflow_project_id, dialogflow_session_id )
        self.language_code = language_code

        # ROS
        rospy.init_node( "InteractionModule" )
        self.rosservice_stop         = rospy.Service( "/interactionmodule/stop",          InteractionModuleServer_Stop,         self.servicehandler_stop )
        self.rossubscriber_status = rospy.Subscriber( "/StatusManager/Status", String, self.callback_status )
        self.rossubscriber_stt = rospy.Subscriber( "/speech_to_text", SpeechRecognitionCandidates, self.callback_stt )

    def callback_stt( self, msg ):
        """
        """
        self.sttbuffer.append( msg.transcript[0] )

    def callback_status( self, msg ):
        """
        """
        #
        self.status = msg.data
        self.isok = True
        #
        if self.status in self.statushandlers.keys():
            self.statushandlerthread = threading.Thread( target = self.statushandlers[ self.status ] )
            self.statushandlerthread.start()
            self.statushandlerthread.detach()

    def setStatus( self ):

        setStatusService = rospy.ServiceProxy( "/StatusManager/Service/set_Status", StatusManagerService_SetStatus )
        ret = setStatusService( StatusManagerService_SetStatusRequest( "" ) )
        

    def servicehandler_stop( self, req ):
        """
        """
        self.isok = False

    def statushandler_default( self ):

    def statushandler_waiting( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )

            #
            # TODO: ret の 状態から,waiting_interaction もしくは guiding へ
            #
            if hogehoge:


    def statushandler_waiting_interaction( self ):

    def statushandler_guiding( self ):

    def statushandler_guiding_halt_waiting( self ):

    def statushandler_guiding_halt_interaction( self ):

    def sendQuery( self, text ):
        text_input  = dialogflow.types.TextInput( text=text, language_code=self.language_code )
        query_input = dialogflow.types.QueryInput( text=text_input )
        response    = self.df_sessionclient.detect_intent( session=session, query_input=query_input )
        return response

    def speakInJapanese( self, text ):
        """
        """
        #TBD
        
