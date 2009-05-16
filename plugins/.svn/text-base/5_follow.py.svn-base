"""
follow.py

PandaSteer plugin demonstrating wander and follow steering behaviors.

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
    """PandaSteer plugin demonstrating wander and follow steering behaviors."""

    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 2
        self.container = ContainerSquare(radius=43)
        self.text = """Follow the leader:

One character wanders while another follows.

The follow behaviour is implemented by applying the previous arrival behaviour
to a point just behind the target each frame. The character steers towards this
point and slows down as it catches up to it, never overtaking the point.

Click with the mouse to control the leader using the arrive behaviour.
"""

    def click(self,pos):
        """Respond to mouse-click."""
        
        self.vehicles[1].arrive(SteerVec(pos.getX(),pos.getY()))

    def restart(self):
        """Start or restart the plugin."""
        
        # Set first character to random position and velocity.        
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)

        # Give second character a random start position and push it off towards the origin.        
        self.vehicles[1]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[1]._velocity = SteerVec(0-self.vehicles[1]._pos.getX(),0-self.vehicles[1]._pos.getY())
        self.vehicles[1]._velocity = self.vehicles[1]._velocity.truncate(self.vehicles[1].maxspeed)
       
        # Tell the first character to follow the second character, and tell the
        # second character to wander.
        self.vehicles[0].follow(self.vehicles[1])
        self.vehicles[1].wander()
