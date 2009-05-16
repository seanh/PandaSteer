"""
seekAndFlee.py

PandaSteer plugin demonstrating seek and flee steering behaviors.

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
    
        self.numVehicles = 2
        self.text = """Seek and Flee:

One character seeks the target while another flees from it. The two characters
are initialised with the same random position and velocity.

Each frame the seeking character computes the direction from itself to its
target and moves in that direction at its maximum speed. The fleeing character
does the same, but moves in the opposite direction.

The seek character does not attempt to slow down as it reaches the target,
and so repeatedly overshoots and turns back, oscillating around the target.

Click with the mouse to move the target.
"""

    def click(self,pos):
        """Respond to mouse-click."""
        
        self.vehicles[0].seek(SteerVec(pos.getX(),pos.getY()))
        self.vehicles[1].flee(SteerVec(pos.getX(),pos.getY()))

    def restart(self):
        """Start or restart the plugin."""
        
        # Set the vehicles to random position and velocity.        
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)
        self.vehicles[1]._pos.setX(self.vehicles[0]._pos.getX())
        self.vehicles[1]._pos.setY(self.vehicles[0]._pos.getY())
        self.vehicles[1]._velocity.setX(self.vehicles[0]._velocity.getX())
        self.vehicles[1]._velocity.setY(self.vehicles[0]._velocity.getY())
       
        # Set the first character to seek the target and the second to flee it.
        self.vehicles[0].seek(SteerVec(0,0))
        self.vehicles[1].flee(SteerVec(0,0))
