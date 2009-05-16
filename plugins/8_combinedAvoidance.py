"""
8_combinedAvoidance.py

PandaSteer plugin demonstrating combined obstacle and collision avoidance with
wandering and pursuit.

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
from steerVec import SteerVec
from pandac.PandaModules import NodePath,Vec3
from pandaSteerPlugin import PandaSteerPlugin
from containers import ContainerSquare

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating combined obstacle and collision
    avoidance with wandering and pursuit.
    """
    
    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
        
        self.numVehicles = 10
        self.obstaclesOn = True
        self.container = ContainerSquare(radius=43)
        self.text = """Obstacle avoidance, collision avoidance, containment and wandering,
        
Ten characters wander while trying to avoid obstacles and collisions and stay
within a set area.

Because obstacle avoidance is prioritised over collision avoidance, characters
sometimes collide when both are too busy steering round an obstacle to avoid
each other."""

    def restart(self):
        """Start or restart the plugin."""
        
        for vehicle in self.vehicles:
            vehicle.maxforce = 0.1
            vehicle.maxspeed = 0.3      
            vehicle._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
            vehicle._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
            vehicle._velocity = vehicle._velocity.truncate(vehicle.maxspeed)
            vehicle.wander()        
        self.vehicles[0].maxspeed = 1
        self.vehicles[0].maxforce = 0.33
        self.vehicles[1].maxspeed = 0.5
        self.vehicles[1].maxforce = 0.2
        self.vehicles[1].follow(self.vehicles[0])
        for vehicle in self.vehicles:
            # Turn on obstacle & collision avoidance steering
            vehicle.avoidObstacles = True
            vehicle.avoidVehicles = True
            
