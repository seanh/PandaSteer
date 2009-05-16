"""
containers.py -- containers that Vehicles can steer to stay within.

Container classes must implement the container protocol:
    
* An isInside(self,p) method that tests whether a point P is inside the
  container
* An instance variable pos with sensible methods getX() and getY().

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
from steerVec import SteerVec
      
class ContainerCircle:
    """A circular vehicle container.
        
    A circle drawn on the XY plane. A point's XY position is tested against the
    circle to see if it is inside, the Z-dimension is ignored.
    
    """
    def __init__(self,pos=None,radius=40,visible=False):
        """Initialise the CircleContainer."""
        
        # Initialise the CircleContainer
        if pos is None: pos = SteerVec(0,0)
        self.pos = pos
        self.radius = radius
        
        if visible:
            # Initialise a CollisionSphere used to represent the CircleContainer
            # graphically (not used for any collision detection). Although the
            # CircleContainer is really 2D, we represent it with a 3D sphere for
            # convenience.
            self.sphere = CollisionSphere(self.pos.getX(),self.pos.getY(),0,
                                          self.radius)
            self.spherenp = render.attachNewNode(CollisionNode('cnode'))
            self.spherenp.node().addSolid(self.sphere)
            self.spherenp.show()
        
    def isInside(self,p):
        """Return True if p is inside this container, False otherwise.
        
        Parameters:
        p -- the point to test, p.X and p.Y should be sensible."""
    
        if (self.pos - p).length() < self.radius: return True
        else: return False

class ContainerSquare:
    """A square vehicle container.
    
    A square drawn on the XY plane. The Z-dimension is ignored.
    
    """
    def __init__(self,pos=None,radius=45):
    
        if pos is None: pos = SteerVec(0,0)
        self.pos = pos
        self.radius=radius
        
    def isInside(self,p):

        if (abs(p.getX() - self.pos.getX()) >= self.radius or
            abs(p.getY() - self.pos.getY()) >= self.radius):
            return False
        else: return True    
