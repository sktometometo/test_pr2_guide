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
                  credential_file,
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
        self.credential_file = credential_file

        # os enviromental variable
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = os.path.abspath( credential_file )

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
        self.isok = True

        if not self.isaliveStatusHandler():
            self.startStatusHandler()

    def callback_status( self, msg ):
        """
        """
        #
        if self.isaliveStatusHandler():
            self.stopStatusHandler()
        #
        self.status = msg.data
        self.isok = True
        self.startStatusHandler()

    def isaliveStatusHandler( self ):
        """
        """
        if self.statushandlerthread is None:
            return False
        else:
            return self.statushandlerthread.is_alive():

    def startStatusHandler( self ):
        """
        """
        if self.status in self.statushandlers.keys():
            self.statushandlerthread = threading.Thread( target = self.statushandlers[ self.status ] )
            self.statushandlerthread.start()
            self.statushandlerthread.detach()
            return True
        else:
            return False

    def stopStatusHandler( self, timeout=10.0 ):

        if self.isaliveStatusHandler():
            self.isok = False
            self.statushandlerthread.join( timeout=timeout )
            if self.statushandlerthread.is_alive():
                self.isok = True
                return False
            else:
                return True
        else:
            return True

    def setStatus( self, status ):

        setStatusService = rospy.ServiceProxy( "/StatusManager/Service/set_Status", StatusManagerService_SetStatus )
        ret = setStatusService( StatusManagerService_SetStatusRequest( status ) )
        return ret.ret

    def callGoToSpotStart( self, target_spot ):
        gotospotStart = rospy.ServiceProxy( "/go_to_spot_server/start_go_to_spot", GoToSpotServiceStart )
        ret = setStatusService( GoToSpotServiceStartRequest( target_spot ) )
        return ret.ret

    def callGoToSpotCancel( self ):
        gotospotCancel = rospy.ServiceProxy( "/go_to_spot_server/cancel_go_to_spot", GoToSpotServiceCancel )
        ret = setStatusService( GoToSpotServiceCancelRequest( ) )
        return ret.ret

    def servicehandler_stop( self, req ):
        """
        """
        self.isok = False

    def statushandler_default( self ):

        self.isok = False

    def statushandler_waiting( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            fulfillment_text = str(ret.query_result.fulfillment_text)

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting_interaction" )

            elif intent_name == "command : halt":

            elif intent_name == "command : abort":

            elif intent_name == "command : resume":

            elif intent_name == "command : guide_start_people":
                self.speakInJapanese( fulfillment_text )
                target_spot = ret.query_result.parameters["Person"]
                self.setStatus( "guiding" )
                callGoToSpotStart( target_spot )

            elif intent_name == "command : guide_start_place":
                self.speakInJapanese( fulfillment_text )
                target_spot = ret.query_result.parameters["Place"]
                self.setStatus( "guiding" )
                callGoToSpotStart( target_spot )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
                # TODO : 詳細な場合分け

        self.isok = False

    def statushandler_waiting_interaction( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            fulfillment_text = str(ret.query_result.fulfillment_text)

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( fulfillment_text )

            elif intent_name == "command : halt":
                self.speakInJapanese( "待機モードに移行します。" )
                self.setStatus( "waiting" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : resume":

            elif intent_name == "command : guide_start_people":
                self.speakInJapanese( fulfillment_text )
                target_spot = ret.query_result.parameters["Person"]
                self.setStatus( "guiding" )
                callGoToSpotStart( target_spot )

            elif intent_name == "command : guide_start_place":
                self.speakInJapanese( fulfillment_text )
                target_spot = ret.query_result.parameters["Place"]
                self.setStatus( "guiding" )
                callGoToSpotStart( target_spot )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
                # TODO : 詳細な場合分け

        self.isok = False

    def statushandler_guiding( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            fulfillment_text = str(ret.query_result.fulfillment_text)

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "guiding_halt_interaction" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : resume":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : people":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : place":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
                # TODO : 詳細な場合分け

        self.isok = False

    def statushandler_guiding_halt_waiting( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            fulfillment_text = str(ret.query_result.fulfillment_text)

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "guiding_halt_interaction" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : resume":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : people":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : place":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
                # TODO : 詳細な場合分け

    def statushandler_guiding_halt_interaction( self ):

        bufstr = ""

        while self.isok and len( self.sttbuffer ) > 0:

            bufstr = self.sttbuffer[0]
            del self.sttbuffer[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            fulfillment_text = str(ret.query_result.fulfillment_text)

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "guiding_halt_interaction" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : resume":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : people":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start : place":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
                # TODO : 詳細な場合分け

    def sendQuery( self, text ):
        text_input  = dialogflow.types.TextInput( text=text, language_code=self.language_code )
        query_input = dialogflow.types.QueryInput( text=text_input )
        response    = self.df_sessionclient.detect_intent( session=session, query_input=query_input )
        return response

    def speakInJapanese( self, text ):
        """
        """
        #TBD
        
if __name__=="__main__":

    im = InteractionModule( sys.argv[1], sys.argv[2], sys.argv[3] )
    rospy.spin()
