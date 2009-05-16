"""
wander.py

PandaSteer plugin demonstrating wander steering behavior.

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
from containers import ContainerSquare

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating wander steering behavior."""

    def __init__(self):
        """Initialise the plugin."""

        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 1
        self.container = ContainerSquare(radius=43)
        self.text = """Wandering and Containment:
 
A character wanders around randomly and stays within a set area.

Combines two steering forces: containment is applied if the character leaves
the container, otherwise wandering is applied. The container is a square shape
slightly smaller than the green platform.

Containment steers the character back toward the center of the container shape.

Wandering randomly alters the steering direction each frame based on the
previous frame's direction, giving smooth curving movement.
"""

    def restart(self):
        """Start or restart the plugin."""
        
        # Set vehicle to a random position and velocity.        
        self.vehicles[0].maxspeed = 0.2
        self.vehicles[0].maxforce = 0.01
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)
       
        # Tell vehicle to wander.
        self.vehicles[0].wander()
