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
        self.servicename_add    = rospy.getparam( "/pr2_guide/servicename/spot_manager/add",    "/spot_manager/add" )
        self.servicename_delete = rospy.getparam( "/pr2_guide/servicename/spot_manager/delete", "/spot_manager/delete" )
        self.servicename_get    = rospy.getparam( "/pr2_guide/servicename/spot_manager/get",    "/spot_manager/get" )
        self.servicename_save   = rospy.getparam( "/pr2_guide/servicename/spot_manager/save",   "/spot_manager/save" ) 
        ##
        self.service_add    = rospy.Service( self.servicename_add,    SpotManagerAdd,    self.servicehandler_add )
        self.service_delete = rospy.Service( self.servicename_detele, SpotManagerDelete, self.servicehandler_delete )
        self.service_get    = rospy.Service( self.servicename_get,    SpotManagerGet,    self.servicehandler_get )
        self.service_save   = rospy.Service( self.servicename_save,   SpotManagerSave,   self.servicehandler_save )
        # SpotManager
        self.spotlist = []
        #
        try:
            self.filepath = rospy.getparam( "/pr2_guide/filename/spot_manager/config" )
            if os.path.exits( self.filepath ):
                self.load_spotlist( self.filepath )
        except KeyError:
            self.filepath = None

    def __del__( self ):
        self.save_spotlist( self.filepath )

    def update_id( self ):
        for index, spot in self.spotlist:
            spot.id = index

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
        if self.path is None:
            return False
        else:
            with open( filepath, "wb" ) as f:
                pickle.dump( self.spotlist, f )
            return True

    def servicehandler_add( self, req ):
        self.spotlist.append( req.spot )
        self.update_id()
        return SpotManagerAddResponse( True )

    def servicehandler_delete( self, req ):
        del self.spotlist[ req.id ]
        self.update_id()
        return SpotManagerDeleteResponse( True )

    def servicehandler_get( self, req ):
        return SpotManagerGetResponse( self.spotlist )

    def servicehandler_save( self, req ):
        self.save_spotlist( self.filepath )
        return SpotManagerSaveResponse( True )


if __name__ == "__main__":
    ss = SpotServer()
    rospy.spin()
