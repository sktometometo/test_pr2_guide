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

class InteractionModule

    def __init__( self, 
                  dialogflow_project_id, 
                  dialogflow_session_id,
                  language_code = "ja" ):

        self.df_sessionclient = dialogflow.SessionClient()
        self.df_session = self.df_sessionclient.session_path( dialogflow_project_id, dialogflow_session_id )
        self.language_code = "ja"

        sel.rosservice = rospy.Service( "", InteractionModule_callhandler, self.rosservicehandler )

        rospy.init_node( "InteractionModule" )


    def sendQuery( self, text ):

        text_input  = dialogflow.types.TextInput( text=text, language_code=self.language_code )
        query_input = dialogflow.types.QueryInput( text=text_input )
        response    = self.df_sessionclient.detect_intent( session=session, query_input=query_input )
        return response

    def speakInJapanese( self, text ):
        """
        """

        #TBD
        
    def rosservicehandler( self, req ): 
        """
        """
        if req.status == "waiting_interaction":

        elif req.status == "guiding_interaction":

        else:
