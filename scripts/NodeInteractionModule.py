#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle

import rospy
import tf
from pr2_guide.srv import *
from pr2_guide.msg import *
from std_msgs.msg import *
from speech_recognition_msgs.msg import *

import dialogflow_v2 as dialogflow

import threading

class InteractionModule:

    def __init__( self ):
        # ROS
        ##
        self.nodename                              = "interaction_module"
        rospy.init_node( self.nodename )

        ##
        self.topicname_status_manager_status       = rospy.get_param( "/pr2_guide/topicname/status_manager/status",
                                                                     "/status_manager/status" )
        self.topicname_speech_to_text              = rospy.get_param( "/pr2_guide/topicname/speech_to_text",
                                                                     "/speech_to_text_pr2_guide" )
        self.topicname_robotsound_jp               = rospy.get_param( "/pr2_guide/topicnamae/robotsound_jp",
                                                                      "/robotsound_jp" )
        self.servicename_status_manager_set_status = rospy.get_param( "/pr2_guide/servicename/status_manager/set_status",
                                                                     "/status_manager/set_status" )
        self.servicename_go_to_spot_start          = rospy.get_param( "/pr2_guide/servicename/go_to_spot/start",
                                                                     "/go_to_spot/start" )
        self.servicename_go_to_spot_cancel         = rospy.get_param( "/pr2_guide/servicename/go_to_spot/cancel",
                                                                     "/go_to_spot/cancel" )
        self.servicename_spot_manager_add          = rospy.get_param( "/pr2_guide/servicename/spot_manager/add",
                                                                     "/spot_manager/add" )
        self.servicename_spot_manager_delete       = rospy.get_param( "/pr2_guide/servicename/spot_manager/delete",
                                                                     "/spot_manager/delete" )
        ##
        self.subscriber_status = rospy.Subscriber( self.topicname_status_manager_status, 
                                                   String,
                                                   self.subscribercallback_status )
        self.subscriber_speech_to_text = rospy.Subscriber( self.topicname_speech_to_text, 
                                                           SpeechRecognitionCandidates, 
                                                           self.subscribercallback_speech_to_text )
        self.publisher_robotsound_jp = rospy.Publisher( self.topicname_robotsound_jp,
                                                        SoundRequest )

        ##
        self.listener = tf.TransformListener()
        
        # member variables initialization for dialog flow
        self.language_code         = rospy.get_param( "/pr2_guide/config/interaction_module/language_code" )
        self.credential_file       = rospy.get_param( "/pr2_guide/filename/interaction_module/credential" )
        self.dialogflow_project_id = rospy.get_param( "/pr2_guide/config/interaction_module/project_id" )
        self.dialogflow_session_id = rospy.get_param( "/pr2_guide/config/interaction_module/session_id" )

        # member variables initialization for status management
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
        self.buffer_text = []
        self.target_spot_stack = []

        # dialog flow
        ## os enviromental variable
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = os.path.abspath( self.credential_file )
        ## initialization for dialogflow
        self.df_sessionclient = dialogflow.SessionsClient()
        self.df_session       = self.df_sessionclient.session_path( self.dialogflow_project_id, self.dialogflow_session_id )

        print( "module initialization finished." )

    def sendQuery( self, text ):
        text_input  = dialogflow.types.TextInput( text=text, language_code=self.language_code )
        query_input = dialogflow.types.QueryInput( text=text_input )
        response    = self.df_sessionclient.detect_intent( session=self.df_session, query_input=query_input )
        return response

    def speakInJapanese( self, text ):
        #TBD
        print( "speaking : " + text )
        msg = SoundRequest()
        msg.command = SoundRequest.PLAY_ONCE
        msg.sound = SoundRequest.SAY
        msg.arg = text
        msg.args = "ja"
        msg.volume = 1.0
        self.publisher_robotsound_jp( msg )

    def isaliveStatusHandler( self ):
        if self.statushandlerthread is None:
            return False
        else:
            return self.statushandlerthread.is_alive()

    def startStatusHandler( self ):
        if self.status in self.statushandlers.keys():
            self.statushandlerthread = threading.Thread( target = self.statushandlers[ self.status ] )
            self.statushandlerthread.start()
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
        setStatusService = rospy.ServiceProxy( self.servicename_status_manager_set_status, StatusManagerSetStatus )
        try:
            ret = setStatusService( StatusManagerSetStatusRequest( status ) )
            return ret.ret
        except rospy.ServiceException:
            return False

    def callGoToSpotStart( self, target_spot ):
        gotospotStart = rospy.ServiceProxy( self.servicename_go_to_spot_start, GoToSpotStart )
        try:
            ret = gotospotStart( GoToSpotStartRequest( target_spot ) )
            return ret.ret
        except rospy.ServiceException:
            return False

    def callGoToSpotCancel( self ):
        gotospotCancel = rospy.ServiceProxy( self.servicename_go_to_spot_cancel, GoToSpotCancel )
        try:
            ret = gotospotCancel( GoToSpotCancelRequest( ) )
            return ret.ret
        except rospy.ServiceException:
            return False
    
    def callSpotManagerAdd( self, name ):
        SpotManagerAddService = rospy.ServiceProxy( self.servicename_spot_manager_add, SpotManagerAdd )
        req = SpotManagerAddRequest()
        # tf
        (translation, rotation) = self.listener.lookupTransform( "base_link", "map",  rospy.Time(0) )
        quat = tf.transformations.quaternion_from_euler( rotation[0], rotation[1], rotation[2] )
        # TODO
        req.spot.header.stamp       = rospy.Time(0)
        req.spot.header.frame_id    = "map"
        req.spot.name               = name
        req.spot.aliases            = [name]
        req.spot.pose.position.x    = translation[0]   
        req.spot.pose.position.y    = translation[1]   
        req.spot.pose.position.z    = translation[2]   
        req.spot.pose.orientation.x = quat[0]   
        req.spot.pose.orientation.y = quat[1]   
        req.spot.pose.orientation.z = quat[2]   
        req.spot.pose.orientation.w = quat[3]   
        try:
            ret = SpotManagerAddService( req )
            return ret.ret
        except rospy.ServiceException:
            return False

    def callSpotManagerDelete( self, name ):
        SpotManagerDeleteService = rospy.ServiceProxy( self.servicename_spot_manager_delete, SpotManagerDelete )
        req = SpotManagerDeleteRequest( name )
        try:
            ret = SpotManagerDeleteService( req )
            return ret.ret
        except rospy.ServiceException:
            return False

    def subscribercallback_speech_to_text( self, msg ):
        print( "subscribercallback_speech_to_text is called" )
        self.buffer_text.append( msg.transcript[0] )
        self.isok = True

        if not self.isaliveStatusHandler():
            self.startStatusHandler()

    def subscribercallback_status( self, msg ):
        print( "subscribercallback_status is called" )
        #
        if self.isaliveStatusHandler():
            self.stopStatusHandler()
        #
        self.status = msg.data
        self.isok = True
        self.startStatusHandler()

    def statushandler_default( self ):
        print( "statushandler_default is called" )

        self.isok = False

    def statushandler_waiting( self ):
        print( "statushandler_waiting is called" )

        bufstr = ""

        while self.isok and len( self.buffer_text ) > 0:

            bufstr = self.buffer_text[0]
            del self.buffer_text[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            temp_ft = ret.query_result.fulfillment_text
            print( "bufstr : " + bufstr )
            print( "type : " + str( type( temp_ft ) ) )
            print( "temp_ft : " + temp_ft )
            print( "intent_name : " + intent_name )
            fulfillment_text = ret.query_result.fulfillment_text

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting_interaction" )

            elif intent_name == "command : abort":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : add_spot":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : delete_spot":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : halt":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : resume":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : start_guiding":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callGoToSpotStart( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                    self.setStatus( "guiding" )
                else:
                    self.speakInJapanese( target_spot + u"への案内に失敗しました。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )

        self.isok = False

    def statushandler_waiting_interaction( self ):
        print( "statushandler_waiting_interaction is called" )

        bufstr = ""

        while self.isok and len( self.buffer_text ) > 0:

            bufstr = self.buffer_text[0]
            del self.buffer_text[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            temp_ft = ret.query_result.fulfillment_text
            print( "bufstr : " + bufstr )
            print( "type : " + str( type( temp_ft ) ) )
            print( "temp_ft : " + temp_ft )
            print( "intent_name : " + intent_name )
            fulfillment_text = ret.query_result.fulfillment_text

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( fulfillment_text )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : add_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerAdd( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの追加に失敗しました。" )

            elif intent_name == "command : delete_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerDelete( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの削除に失敗しました。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( "待機モードに移行します。" )
                self.setStatus( "waiting" )

            elif intent_name == "command : resume":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "command : start_guiding":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callGoToSpotStart( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                    self.setStatus( "guiding" )
                else:
                    self.speakInJapanese( target_spot + u"への案内に失敗しました。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )

        self.isok = False

    def statushandler_guiding( self ):
        print( "statushandler_guiding is called" )

        bufstr = ""

        while self.isok and len( self.buffer_text ) > 0:

            bufstr = self.buffer_text[0]
            del self.buffer_text[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            temp_ft = ret.query_result.fulfillment_text
            print( "bufstr : " + bufstr )
            print( "type : " + str( type( temp_ft ) ) )
            print( "temp_ft : " + temp_ft )
            print( "intent_name : " + intent_name )
            fulfillment_text = ret.query_result.fulfillment_text

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : add_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerAdd( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの追加に失敗しました。" )

            elif intent_name == "command : delete_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerDelete( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの削除に失敗しました。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.callGoToSpotCancel() 
                self.setStatus( "guiding_halt_interaction" )

            elif intent_name == "command : resume":
                print( "intent_name : " + intent_name + ", do nothing." )

            elif intent_name == "guiding : start_guiding":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )

        self.isok = False

    def statushandler_guiding_halt_waiting( self ):
        print( "statushandler_guiding_halt_waiting is called" )

        bufstr = ""

        while self.isok and len( self.buffer_text ) > 0:

            bufstr = self.buffer_text[0]
            del self.buffer_text[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            temp_ft = ret.query_result.fulfillment_text
            print( "bufstr : " + bufstr )
            print( "type : " + str( type( temp_ft ) ) )
            print( "temp_ft : " + temp_ft )
            print( "intent_name : " + intent_name )
            fulfillment_text = ret.query_result.fulfillment_text

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : add_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerAdd( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの追加に失敗しました。" )

            elif intent_name == "command : delete_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerDelete( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの削除に失敗しました。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "guiding_halt_interaction" )

            elif intent_name == "command : resume":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start_guiding":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )

    def statushandler_guiding_halt_interaction( self ):
        print( "statushandler_guiding_halt_interaction is called" )

        bufstr = ""

        while self.isok and len( self.buffer_text ) > 0:

            bufstr = self.buffer_text[0]
            del self.buffer_text[0]
            ret = self.sendQuery( bufstr )
            intent_name = ret.query_result.intent.display_name
            temp_ft = ret.query_result.fulfillment_text
            print( "bufstr : " + bufstr )
            print( "type : " + str( type( temp_ft ) ) )
            print( "temp_ft : " + temp_ft )
            print( "intent_name : " + intent_name )
            fulfillment_text = ret.query_result.fulfillment_text

            if intent_name == "Default Fallback Intent":
                self.speakInJapanese( fulfillment_text )
                # DO NOTHING

            elif intent_name == "Default Welcome Intent":
                self.speakInJapanese( fulfillment_text )

            elif intent_name == "command : abort":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "waiting" )

            elif intent_name == "command : add_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerAdd( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの追加に失敗しました。" )

            elif intent_name == "command : delete_spot":
                target_spot = ret.query_result.parameters["Spot"]
                if self.callSpotManagerDelete( target_spot ):
                    self.speakInJapanese( fulfillment_text )
                else:
                    self.speakInJapanese( "スポットの削除に失敗しました。" )

            elif intent_name == "command : halt":
                self.speakInJapanese( fulfillment_text )
                self.setStatus( "guiding_halt_waiting" )

            elif intent_name == "command : resume":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            elif intent_name == "guiding : start_guiding":
                self.speakInJapanese( "道案内動作中です。他の案内をご希望される場合は、一度中止してください。" )

            else:
                self.speakInJapanese( fulfillment_text )
                self.speakInJapanese( "未知のインテントが返ってきています。" )
        
if __name__=="__main__":

    im = InteractionModule( )
    rospy.spin()
