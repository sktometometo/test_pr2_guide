#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle
from pr2_guide.srv import *
from pr2_guide.msg import *

class SpotServer:

    def __init__( self,
                  filetpath=None,
                  nodename="spot_manager", 
                  servicename_add="/spot_manager/add",
                  servicename_delete="/spot_manager/delete",
                  servicename_get="/spot_manager/get",
                  servicename_save="/spot_manager/save" ):
        # ROS
        self.nodename = nodename
        self.servicename_add = servicename_add
        self.servicename_delete = servicename_delete
        self.servicename_get = servicename_get
        self.servicename_save = servicename_save

        rospy.init_node( self.nodename )
        self.service_add    = rospy.Service( self.servicename_add,    SpotManagerAdd,    self.servicehandler_add )
        self.service_delete = rospy.Service( self.servicename_detele, SpotManagerDelete, self.servicehandler_delete )
        self.service_get    = rospy.Service( self.servicename_get,    SpotManagerGet,    self.servicehandler_get )
        self.service_save   = rospy.Service( self.servicename_save,   SpotManagerSave,   self.servicehandler_save )

        # SpotManager
        self.spotlist = []

        if filepath is None:
            self.filepath = ""
        else:
            self.filepath = filepath

        if os.path.exits( self.filepath ):
            self.load_spotlist( self.filepath )

    def __del__( self ):
        self.save_spotlist( self.filepath )

    def update_id( self ):
        for index, spot in self.spotlist:
            spot.id = index

    def load_spotlist( self, filepath ):
        if os.path.exists( os.path.abspath( filepath ) ):
            with open( filepath, "rb" ) as f:
                self.spotlist = pickle.load( f )
            return True
        else:
            return False

    def save_spotlist( self, filepath ):
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

def main():
    if len( sys.argv ) > 1:
        ss = SpotServer( os.path.abspath( sys.argv[1] ) )
    else:
        ss = SpotServer()
    rospy.spin()

if __name__ == "__main__":
    main()
