"""
pursueAndEvade.py

PandaSteer plugin demonstrating pursue and evade steering behaviors.

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
    """PandaSteer plugin demonstrating pursue and evade steering behaviors."""

    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 3
        self.text = """Pursue and Evade:

One character pursues the drone character, another evades it. The pursuing and
evading characters are initialised with the same random position and velocity.

Pursuit and evasion use a simple, linear, velocity-based predictor to predict
the position of the target a short time in the future, then seek or flee from
the predicted position.

The pursuing character does not slow down as it catches up with the target so
may overtake the target as it steers towards the target's predicted future
position.

Click with the mouse to control the drone character using the seek behaviour.
"""

    def click(self,pos):
        """Respond to mouse-click."""
        
        self.vehicles[2].seek(SteerVec(pos.getX(),pos.getY()))

    def restart(self):
        """Start or restart the plugin."""
        
        # Set the first two vehicles to random position and velocity.        
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)
        self.vehicles[1]._pos.setX(self.vehicles[0]._pos.getX())
        self.vehicles[1]._pos.setY(self.vehicles[0]._pos.getY())
        self.vehicles[1]._velocity.setX(self.vehicles[0]._velocity.getX())
        self.vehicles[1]._velocity.setY(self.vehicles[0]._velocity.getY())

        # Give third vehicle a random start position and push it off towards the origin.        
        self.vehicles[2]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[2]._velocity = SteerVec(0-self.vehicles[2]._pos.getX(),0-self.vehicles[2]._pos.getY())
        self.vehicles[2]._velocity = self.vehicles[2]._velocity.truncate(self.vehicles[2].maxspeed)
       
        # Set first vehicle to pursue and second vehicle to evade third vehicle
        self.vehicles[0].pursue(self.vehicles[2])
        self.vehicles[1].evade(self.vehicles[2])
