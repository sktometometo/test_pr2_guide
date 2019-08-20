#!/usr/bin/env python
# coding: utf-8

import yaml
import sys
import os
import ros

class SpotTree(object):

    def __init__( self, filename=None ):
        """
        """

        self.rootNode = None
        self.header = None

        if filename is not None:
            self.loadYaml( filename )

    def loadYaml( self, filename ):
        """
        """

        with open( filename ) as f:
            self.yamldata = yaml.load( f )
            print( type(self.yamldata) )
            self.rootNode = SpotTreeNode( self.yamldata["data"] )

    def listName( self ):
        """
        """
        return self.rootNode.listName()

    def listText( self ):
        """
        """
        return self.rootNode.listText()

    def searchName( self, name, iscontained=False ):
        """
        """
        return self.rootNode.searchName( name, iscontained )

    def searchText( self, text, iscontained=False ):
        """
        """
        return self.rootNode.searchText( text, iscontained )

class SpotTreeNode(object):

    def __init__( self, yamldata=None ):
        """
        """

        self.nodename = None
        self.text = None
        self.aliases = []
        self.children = []

        if yamldata is not None:
            self.createFromYamlNode( yamldata )

    def getName( self ):
        """
        """
        return self.nodename


    def getText( self ):
        """
        """
        return self.text

    def getAliases( self ):
        """
        """
        return self.aliases

    def createFromYamlNode( self, yamldata ):
        """
        """

        try:
            if "name" in yamldata.keys() \
                and "text" in yamldata.keys() \
                and "aliases" in yamldata.keys() \
                and "children" in yamldata.keys():

                self.nodename = yamldata["name"]
                self.text = yamldata["text"]
                self.aliases = yamldata["aliases"]

                for yamldatanode in yamldata["children"]:
                    self.children.append( SpotTreeNode( yamldatanode ) )

            else:
                # error
                raise ValueError("hoge")
        except:
            print( "SpotTreeNode construction error" )

    def listName( self ):
        """
        """
        ans = [ self.nodename ]
        for x in self.children:
            ans += x.listName()
        return ans

    def listText( self ):
        """
        """
        ans = [ self.text ]
        for x in self.children:
            ans += x.listText()
        return ans
        
    def searchName( self, name, iscontained=False ):
        """
        """
        if iscontained:
            if self.nodename in name:
                ans = [ self ]
                for x in self.children:
                    ans += x.searchName( name, iscontained )
                return ans
            else:
                ans = [ ]
                for x in self.children:
                    ans += x.searchName( name, iscontained )
                return ans
        else:
            if name == self.nodename:
                ans = [ self ]
                for x in self.children:
                    ans += x.searchName( name, iscontained )
                return ans
            else:
                ans = [ ]
                for x in self.children:
                    ans += x.searchName( name, iscontained )
                return ans

    def searchText( self, text, iscontained=False ):
        """
        """
        if iscontained:
            if self.text in text:
                ans = [ self ]
                for x in self.children:
                    ans += x.searchText( text, iscontained )
                return ans
            else:
                ans = []
                for x in self.children:
                    ans += x.searchText( text, iscontained )
                return ans
        else:
            if text == self.text:
                ans = [ self ]
                for x in self.children:
                    ans += x.searchText( text, iscontained )
                return ans
            else:
                ans = []
                for x in self.children:
                    ans += x.searchText( text, iscontained )
                return ans

def main():
    """
    """


if __name__ == "__main__":
    main()
