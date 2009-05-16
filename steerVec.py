"""
steerVec.py -- an extension of Panda3D's Vec2 class with some vector methods
needed for steering behaviors.

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

from pandac.PandaModules import Vec2, deg2Rad
from math import sqrt,atan2,cos,sin

class SteerVec(Vec2):
    """SteerVec extends Panda's standard Vec2 class and adds some vector
    methods needed to implement steering behaviours."""
    
    def __init__(self,x=0,y=0):
        """Initialise the SteerVec. Nothing to do, just calls Vec2.__init__."""
        
        Vec2.__init__(self,x,y)
        
    def truncate(self,scalar):
        """Return this vector truncated by the scalar value 'scalar.' The
        returned vector will have length <= 'scalar' and will have the same
        direction as this vector."""
        
        scalarSquared = scalar * scalar
        lengthSquared = self.dot(self)
        if (lengthSquared <= scalarSquared):
            # The vector is already shorter in length than the scalar.
            return self
        else:
            return self * ( scalar / sqrt(lengthSquared))

    def rotate(self,rotationAngle):
        """Return this vector rotated by 'rotationAngle' degrees. The returned
        vector will have the same length as this vector, but a different
        direction."""
    
        angle = atan2(self.getY(), self.getX())
        angle += deg2Rad(rotationAngle)
        x = cos(angle)
        y = sin(angle)
        return SteerVec(x,y)

    def parallelComponent(self,unit):
        """Return the component (SteerVec) of this vector that is parallel
        to 'unit' (SteerVec). 'unit' must be a unit vector (a vector of length
        1)."""       
                
        projection = self.dot(unit)
        return unit * projection

    def perpendicularComponent(self,unit):
        """Return the component (SteerVec) of this vector that is perpendicular
        to 'unit' (SteerVec). 'unit' must be a unit vector (a vector of length
        1)."""       
        
        parallelComponent = self.parallelComponent(unit)
        perpendicularComponent = self - parallelComponent
        return perpendicularComponent

    # Need to override all the Vec2 methods that return a Vec2, and return
    # a SteerVec instead.

    def __mul__(self,other):
        vec2 = Vec2.__mul__(self,other)
        return SteerVec(vec2.getX(),vec2.getY())

    def __add__(self,other):
        vec2 = Vec2.__add__(self,other)
        return SteerVec(vec2.getX(),vec2.getY())
        
    def __sub__(self,other):
        vec2 = Vec2.__sub__(self,other)
        return SteerVec(vec2.getX(),vec2.getY())
    
    def __div__(self,other):
        vec2 = Vec2.__div__(self,other)
        return SteerVec(vec2.getX(),vec2.getY())
        
    def __neg__(self):
        vec2 = Vec2.__neg__(self)
        return SteerVec(vec2.getX(),vec2.getY())

    def __str__(self):
        return '('+str(self.getX())[:3]+','+str(self.getY())[:3]+')'
