"""
collisionAvoidance.py

PandaSteer plugin demonstrating unaligned collision avoidance steering.

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
from pandaSteerPlugin import PandaSteerPlugin
from pandac.PandaModules import NodePath,Vec3
from containers import ContainerSquare

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating unaligned collision avoidance steering.
    """
    
    def __init__(self):
        """Initialise the plugin."""

        PandaSteerPlugin.__init__(self)  
    
        self.numVehicles = 12
        self.container = ContainerSquare(radius=43)
        self.text = """Wandering pedestrians (unaligned collision avoidance, containment and wandering):

Twelve wandering characters avoid colliding with each other. Each character
compares its velocity with those of its neighbours to predict future collisions,
and steers to avoid colliding.

If an imminent or glancing collision between two characters is predicted the
characters steer away from each other. If the predicted collision is head-on
the characters steer away from the point of collision. If the collision is
perpendicular then the slower character steers behind the faster.
"""

    def restart(self):
        """Start or restart the plugin."""
        
        for vehicle in self.vehicles:
            vehicle.maxforce = 0.1
            vehicle.maxspeed = 0.2      
            vehicle._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
            vehicle._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
            vehicle._velocity = vehicle._velocity.truncate(vehicle.maxspeed)
            vehicle.wander()
            # Turn on collision avoidance steering
            vehicle.avoidVehicles = True
        self.vehicles[4].maxspeed = 1
