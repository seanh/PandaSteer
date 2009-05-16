"""
obstacles.py -- obstacle volumes that Vehicles can steer to avoid.

To be avoided by Vehicles an obstacle must:

* Add itself as a 'from' object to the CollisionTraverser 'cTrav' in
  vehicle.py, using the CollisionHandler 'obstacleHandler' also in vehicle.py
* and set its 'from' collision mask to 'obstacleMASK' in vehicle.py

See examples below.

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
# Panda imports
from pandac.PandaModules import * 
# Custom imports
import vehicle as V

class SphereObstacle:
    """A CollisionSphere that registers itself with vehicle.py's 
    obstacleHandler, so that Vehicles will treat it as an obstacle."""

    def __init__(self,x=0,y=0,z=0,radius=1,show=True):
        """Initialise the CollisionSphere."""
    
        self._pos = Vec2(x,y)
        self.radius = radius
        cs = CollisionSphere(x,y,z,radius)
        self.cnp = render.attachNewNode(CollisionNode('SphereObstacle'))
        self.cnp.node().addSolid(cs)
        self.cnp.node().setIntoCollideMask(V.offMASK)
        self.cnp.node().setFromCollideMask(V.obstacleMASK)
        V.cTrav.addCollider(self.cnp,V.obstacleHandler)
        V.obstacles.append(self)
        if show: self.cnp.show()
    
    def destroy(self):
        """Remove the CollisionSphere from the global CollisionTraverser and
        detach the NodePath from the scene graph. After executing this method,
        the SphereObstacle is ready to be garbage-collected by Python once any
        remaining references to it in the user module are destroyed."""
    
        V.cTrav.removeCollider(self.cnp)
        V.obstacles.remove(self)
        self.cnp.detachNode()
