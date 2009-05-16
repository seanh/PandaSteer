"""
obstacleAvoidance.py

PandaSteer plugin demonstrating wander and pursue steering behaviors combined
with obstacle avoidance.

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

# Python imports.
import random
# Panda imports.
from pandac.PandaModules import NodePath,Vec3
# Custom imports.
from steerVec import SteerVec
from pandaSteerPlugin import PandaSteerPlugin
from containers import ContainerSquare

class Plugin(PandaSteerPlugin):
    """PandaSteer plugin demonstrating wander and pursue steering behaviors
       combined with obstacle avoidance."""

    def __init__(self):
        """Initialise the plugin."""
    
        PandaSteerPlugin.__init__(self)
    
        self.numVehicles = 2
        self.obstaclesOn = True
        self.container = ContainerSquare(radius=43)
        self.text = """Obstacle Avoidance:

One character wanders while the other follows. Both try to avoid obstacles and
remain on the platform (but do not try to avoid eachother).

Obstacles are detected using a CollisionTube around each character that varies
in length frame-by-frame according to the character's speed. Each frame the
nearest obstacle in collision with the CollisionTube is chosen and a steering
force to turn the character away from the obstacle is applied, first gently
then stronger each frame until the collision is avoided. A braking force is
also applied as an obstacle approaches. 

When an obstacle is close to the edge of the container, containment steering
may steer the character into the obstacle and leave no time for obstacle
avoidance steering to kick in.

Click with the mouse to control the pursuing character using the arrive
behaviour.
"""

    def restart(self):
        """Start or restart the plugin."""
        
        self.vehicles[0]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[0]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[0]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)

        self.vehicles[1]._pos = SteerVec( (random.random()-0.5)*100, (random.random()-0.5)*100 )
        self.vehicles[1]._velocity = SteerVec( (random.random()-0.5)*2, (random.random()-0.5)*2 )
        self.vehicles[1]._velocity = self.vehicles[0]._velocity.truncate(self.vehicles[0].maxspeed)

        self.vehicles[1]._pos = SteerVec(-15,-15)
        self.vehicles[1]._velocity = SteerVec(0.1,0.1)
        self.vehicles[1].wander()
        
        self.vehicles[1].maxforce = 0.05
        self.vehicles[1].maxspeed = 0.55

        self.vehicles[0].follow(self.vehicles[1])
        self.vehicles[0].maxforce = 0.05
        self.vehicles[0].maxspeed = 0.45
        
        for vehicle in self.vehicles:
            # Turn on obstacle avoidance steering
            vehicle.avoidObstacles = True

    def click(self,pos):
        """Respond to mouse-click."""
        
        self.vehicles[0].arrive(SteerVec(pos.getX(),pos.getY()))
        
