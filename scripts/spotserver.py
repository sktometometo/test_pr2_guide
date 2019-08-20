#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle
from pr2_guide.srv import *
from pr2_guide.msg import *

class SpotServer:

    def __init__( self, filetpath=None ):
        """
        """
        rospy.init_node( "spotserver" )
        
        self.serv_add_spot = rospy.Service( "/spotserver/add_spot", SpotServer_AddSpot, self.handler_add_spot )
        self.serv_delete_spot = rospy.Service( "/spotserver/delete_spot", SpotServer_DeleteSpot, self.handler_delete_spot )
        self.serv_get_spots = rospy.Service( "/spotserver/get_spots", SpotServer_GetSpots, self.handler_get_spots )
        self.serv_save = rospy.Service( "/spotserver/save_file", SpotServer_SaveFile, self.handler_save_file )

        self.spotlist = []

        if filepath is None:
            self.filepath = ""
        else:
            self.filepath = filepath

            if os.path.exits( filepath ):
                self.load_spotlist( filepath )

    def update_id( self ):
        """
        """
        for index, spot in self.spotlist:
            spot.id = index

    def handler_add_spot( self, req ):
        """
        """
        self.spotlist.append( req.spot )
        self.update_id()
        return SpotServer_AddSpotResponse( True )

    def handler_delete_spot( self, req ):
        """
        """
        del self.spotlist[ req.id ]
        self.update_id()
        return SpotServer_DeleteSpotResponse( True )

    def handler_get_spots( self, req ):
        """
        """
        return SpotServer_GetSpotsResponse( SpotArray( self.spotlist ) )

    def handler_save_file( self, req ):
        """
        """
        self.save_spotlist( self.filepath )
        return SpotServer_SaveFile( True )

    def load_spotlist( self, filepath ):
        """
        """
        with open( filepath, "rb" ) as f:
            self.spotlist = pickle.load( f )

    def save_spotlist( self, filepath ):
        """
        """
        with open( filepath, "wb" ) as f:
            pickle.dump( self.spotlist, f )

def main():
    
    ss = SpotServer( os.path.abspath( sys.argv[1] ) )
    rospy.spin()

if __name__ == "__main__":
    main()
