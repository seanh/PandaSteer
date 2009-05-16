"""
arrive.py

PandaSteer plugin demonstrating arrive steering behavior.

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

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating seek and flee steering behaviors."""

    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 1
        self.text = """Going from A to B (arrival):

The character begins at a random position moving at full speed in a random
direction and must move towards the target and arrive there at zero velocity.

Arrival is an enhancement of the seek behaviour. As well as steering towards
the target a braking force is applied when within range of the target, slowing
the character to a stop at the target.

Click with the mouse to move the target. As you can see, this steering
behaviour implements a point-and-click mouse-controlled avatar. It can also be
used to tell a non-player character to go to some location.
"""

    def click(self,pos):
        """Respond to mouse-click."""
        
        self.vehicles[0].arrive(SteerVec(pos.getX(),pos.getY()))

    def restart(self):
        """Start or restart the plugin."""

        # Set vehicle to a random position and velocity.        
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)
       
        # Tell vehicle to arrive at the origin.
        self.vehicles[0].arrive(SteerVec(0,0))
