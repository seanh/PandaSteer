"""
9_pathFollowing.py

PandaSteer plugin demonstrating path following steering behavior.

Copyright (c) 2007 Sean Hammond seanh@sdf.lonestar.org

    This file is part of PandaSteer.

    PandaSteer is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    PandaSteer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PandaSteer; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
"""

import random
from draw import Draw
from steerVec import SteerVec
from pandaSteerPlugin import PandaSteerPlugin
from pandac.PandaModules import Vec2,Vec3,NodePath
from path import Path

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating seek and flee steering behaviors."""
    
    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 6
        self.text = """"""
        self.obstaclesOn = True
        self.text = """Path Following:

Six vehicles follow two overlapping paths at different speeds.

Characters can steer around obstacles and other characters that intersect their
path (but cannot handle an obstacle that covers a waypoint).

This is a very primitive path following behaviour. A path is a list of 2D
waypoints. Characters first seek toward the nearest waypoint on the path, then
seek toward each following waypoint in turn. Because characters don't anticipate
the change of direction when passing a waypoint they may overshoot if moving too
fast, then have to find their way back to the path. When reaching the end of the
path, characters can either loop back to the first waypoint or can arrive and
stop at the final waypoint.
"""

        
    def restart(self):
        """Start or restart the plugin."""

        self.draw = Draw() # Utility for drawing 2D lines and shapes in 3-space
        path = Path([Vec2(40,40),Vec2(40,-40),Vec2(-40,40),Vec2(-40,-40)])
        for vehicle in self.vehicles[:3]:
            vehicle._pos = SteerVec((random.random()-0.5)*100,(random.random()-0.5)*100)
            vehicle._velocity = SteerVec((random.random()-0.5)*2,(random.random()-0.5)*2)
            vehicle.followPath(path,loop=True)
        c=(.6,.6,.6,.3)
        t=1
        z=1.5
        self.draw.drawLine(Vec3(path[0].getX(),path[0].getY(),z),Vec3(path[1].getX(),path[1].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[1].getX(),path[1].getY(),z),Vec3(path[2].getX(),path[2].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[2].getX(),path[2].getY(),z),Vec3(path[3].getX(),path[3].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[3].getX(),path[3].getY(),z),Vec3(path[0].getX(),path[0].getY(),z),color=c,thickness=t)
        path = Path([Vec2(-40,-40),Vec2(40,-40),Vec2(-40,40),Vec2(40,40)])
        for vehicle in self.vehicles[3:]:
            vehicle._pos = SteerVec((random.random()-0.5)*100,(random.random()-0.5)*100)
            vehicle._velocity = SteerVec((random.random()-0.5)*2,(random.random()-0.5)*2)
            vehicle.followPath(path,loop=True)
        self.draw.drawLine(Vec3(path[0].getX(),path[0].getY(),z),Vec3(path[1].getX(),path[1].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[1].getX(),path[1].getY(),z),Vec3(path[2].getX(),path[2].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[2].getX(),path[2].getY(),z),Vec3(path[3].getX(),path[3].getY(),z),color=c,thickness=t)
        self.draw.drawLine(Vec3(path[3].getX(),path[3].getY(),z),Vec3(path[0].getX(),path[0].getY(),z),color=c,thickness=t)        
        for vehicle in self.vehicles:
            vehicle.avoidObstacles = True
            vehicle.avoidVehicles = True
            vehicle.maxforce = 0.1
            vehicle.maxspeed = 0.5 + (random.random()/2)            
        node = self.draw.create()
        np = NodePath(node)   
        np.reparentTo(render)

