#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle
from pr2_guide.srv import *
from pr2_guide.msg import *

class SpotServer:

    def __init__( self ):
        # ROS
        ##
        self.nodename = "spot_manager"
        rospy.init_node( self.nodename )
        ##
        self.servicename_add    = rospy.get_param( "/pr2_guide/servicename/spot_manager/add",    "/spot_manager/add" )
        self.servicename_delete = rospy.get_param( "/pr2_guide/servicename/spot_manager/delete", "/spot_manager/delete" )
        self.servicename_get    = rospy.get_param( "/pr2_guide/servicename/spot_manager/get",    "/spot_manager/get" )
        self.servicename_save   = rospy.get_param( "/pr2_guide/servicename/spot_manager/save",   "/spot_manager/save" ) 
        ##
        self.service_add    = rospy.Service( self.servicename_add,    SpotManagerAdd,    self.servicehandler_add )
        self.service_delete = rospy.Service( self.servicename_delete, SpotManagerDelete, self.servicehandler_delete )
        self.service_get    = rospy.Service( self.servicename_get,    SpotManagerGet,    self.servicehandler_get )
        self.service_save   = rospy.Service( self.servicename_save,   SpotManagerSave,   self.servicehandler_save )
        # SpotManager
        self.spotlist = {}
        #
        try:
            self.filepath = rospy.get_param( "/pr2_guide/filename/spot_manager/config", "~/spotserver.pickle" )
            if os.path.exists( self.filepath ):
                self.load_spotlist( self.filepath )
            print( "initialize finished." )
        except KeyError:
            self.filepath = None

    def __del__( self ):
        self.save_spotlist( self.filepath )

    def load_spotlist( self, filepath ):
        if self.filepath is None:
            return False
        if os.path.exists( os.path.abspath( filepath ) ):
            with open( filepath, "rb" ) as f:
                self.spotlist = pickle.load( f )
            return True
        else:
            return False

    def save_spotlist( self, filepath ):
        if self.filepath is None:
            return False
        else:
            with open( filepath, "wb" ) as f:
                pickle.dump( self.spotlist, f )
            return True

    def servicehandler_add( self, req ):
        print( "add service called" )
        spot = req.spot
        if spot.name in self.spotlist.keys():
            return SpotManagerAddResponse( False )
        self.spotlist[ spot.name ] = spot
        return SpotManagerAddResponse( True )

    def servicehandler_delete( self, req ):
        print( "del service called" )
        if req.name in self.spotlist.keys():
            self.spotlist.pop( req.name )
        return SpotManagerDeleteResponse( True )

    def servicehandler_get( self, req ):
        print( "get service called" )
        return SpotManagerGetResponse( list( self.spotlist.values() ) )

    def servicehandler_save( self, req ):
        print( "save service called" )
        ret = self.save_spotlist( self.filepath )
        return SpotManagerSaveResponse( ret )


if __name__ == "__main__":
    ss = SpotServer()
    rospy.spin()
