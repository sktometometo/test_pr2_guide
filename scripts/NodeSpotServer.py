#!/usr/bin/env python
# coding: utf-8

import os
import sys
import rospy
import pickle
import yaml
from pr2_guide.srv import *
from pr2_guide.msg import *

def str2unicode_in_yaml( yamldata ):

    if type( yamldata ) == list:
        return map( str2unicode_in_yaml, yamldata )
    elif type( yamldata ) == dict:
        return dict( [ (key, str2unicode_in_yaml( val )) for key,val in yamldata.items() ]  )
    #elif type( yamldata ) == str:
    #    return yamldata.decode( "utf-8" )
    elif type( yamldata ) == unicode:
        return yamldata.encode( "utf-8" )
    else:
        return yamldata

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
        self.filepath = os.path.expandvars( os.path.expanduser( rospy.get_param( "/pr2_guide/filename/spot_manager/config", "~/spotserver.yaml" ) ) )
        print( "self.filepath is " + self.filepath )
        if os.path.exists( self.filepath ):
            self.load_spotlist( self.filepath )
        else:
            print( self.filepath + " does not exist." )
        print( "initialize finished." )

    def __del__( self ):
        self.save_spotlist( self.filepath )

    def load_spotlist( self, filepath ):
        print( "load_spotlist is called. filepath is " + filepath )
        if self.filepath is None:
            return False
        if os.path.exists( os.path.abspath( filepath ) ):
            with open( filepath, "r" ) as f:
                yamldata = yaml.load( f, Loader=yaml.SafeLoader )
                yamldata = str2unicode_in_yaml( yamldata )
                tempspotlist = yamldata["spots"]

                if type( tempspotlist ) == dict:
                    self.spotlist = tempspotlist

                elif type( tempspotlist ) == list:
                    self.spotlist = {}
                    for x in tempspotlist:
                        self.spotlist[x["name"]] = x

            return True
        else:
            return False

    def save_spotlist( self, filepath ):
        print( "save_spotlist is called. filepath is " + filepath )
        if self.filepath is None:
            return False
        else:
            with open( filepath, "w" ) as f:
                yaml.safe_dump( { "spots":self.spotlist }, f, encoding="utf-8", allow_unicode=True )
            return True

    def yaml2spot( self, yamled_spot ):
        if "header" not in yamled_spot.keys() \
            or "name" not in yamled_spot.keys() \
            or "aliases" not in yamled_spot.keys() \
            or "pose" not in yamled_spot.keys() :
            return None
            # TODO : Exception のクラスを作ってなげるようにする
        else:
            ret_spot = Spot()
            ret_spot.header.frame_id    = yamled_spot["header"]["frame_id"]
            ret_spot.name               = yamled_spot["name"]
            ret_spot.aliases            = yamled_spot["aliases"]
            ret_spot.pose.position.x    = yamled_spot["pose"]["position"]["x"]
            ret_spot.pose.position.y    = yamled_spot["pose"]["position"]["y"]
            ret_spot.pose.position.x    = yamled_spot["pose"]["position"]["z"]
            ret_spot.pose.orientation.x = yamled_spot["pose"]["orientation"]["x"]
            ret_spot.pose.orientation.y = yamled_spot["pose"]["orientation"]["y"]
            ret_spot.pose.orientation.z = yamled_spot["pose"]["orientation"]["z"]
            ret_spot.pose.orientation.w = yamled_spot["pose"]["orientation"]["w"]
            return ret_spot

    def spot2yaml( self, spot_obj ):
        if type( spot_obj ) is not type( Spot ):
            return None
        else:
            return { "header"  : 
                        { "seq" : Spot.header.seq, 
                          "stamp" : { "secs" : Spot.header.stamp.secs, "nsecs" : Spot.header.stamp.nsecs },
                          "frame_id" : Spot.header.frame_id },
                     "name"    : Spot.name,
                     "aliases" : Spot.aliases,
                     "pose"    : 
                        { "position"    : 
                              { "x" : Spot.pose.position.x,
                                "y" : Spot.pose.position.y,
                                "z" : Spot.pose.position.z },
                          "orientation" :
                              { "x" : Spot.pose.orientation.x,
                                "y" : Spot.pose.orientation.y,
                                "z" : Spot.pose.orientation.z,
                                "w" : Spot.pose.orientation.w }
                              }
                        }


    def servicehandler_add( self, req ):
        print( "add service called" )
        spot = req.spot
        if spot.name in self.spotlist.keys():
            return SpotManagerAddResponse( False )
        self.spotlist[ spot.name ] = self.spot2yaml( spot )
        return SpotManagerAddResponse( True )

    def servicehandler_delete( self, req ):
        print( "del service called" )
        if req.name in self.spotlist.keys():
            self.spotlist.pop( req.name )
        return SpotManagerDeleteResponse( True )

    def servicehandler_get( self, req ):
        print( "get service called" )
        return SpotManagerGetResponse( map( self.yaml2spot, list( self.spotlist.values() ) ) )

    def servicehandler_save( self, req ):
        print( "save service called" )
        ret = self.save_spotlist( self.filepath )
        return SpotManagerSaveResponse( ret )


if __name__ == "__main__":
    ss = SpotServer()
    rospy.spin()
