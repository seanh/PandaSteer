"""
   draw.py -- a library of functions for drawing coloured 2D and 3D lines
   and shapes in Panda3D.

   Usage: the example at the bottom of this file shows how to use Draw objects
   to draw static and moving lines and shapes and shows how to annotate moving
   shapes with text.

   TODO: drawXZGrid and drawXZCircle could be generalised by adding drawXY and
   drawYZ equivalents and adding an optional local space definition (NodePath?)
   as an argument, to allow drawing with the position, orientation and scale of
   an arbitrary local space.
   
Copyright (c) 2007 Sean Hammond seanh@sdf.lonestar.org

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.   
"""

from pandac.PandaModules import *
import sys,math

class Draw(LineSegs):

    def __init__(self):
        LineSegs.__init__(self)

    def drawLine(self,startPoint,endPoint,color=None,thickness=None):
        """Draw a line from startPoint to endPoint using the given color and
        thickness.
        
        color: 4-tuple of floats between 0 and 1: r,g,b,a
        thickness: float
        """
        
        if color is None: color = (1,1,1,1)
        if thickness is None: thickness = 1
                
        self.setColor(*color)
        self.setThickness(thickness)
        self.moveTo(startPoint)
        self.drawTo(endPoint)
    
    def drawAxes(self,size=None,color=None,thickness=None,np=None):
        """Draw the three axes of  NodePath np: three lines parallel to the
        basis vectors of the NodePath's local space, centered at its origin, of
        lengths given by the coordinates of "size" (Vec3). np specifies the
        position, orientation and scale of the axes in the global space. If no
        NodePath argument is passed, draw the axes of the global space.
        """
     
        if size is None: size = Vec3(100,100,100)
        
        if np is None:
            self.drawLine(Vec3(0,0,0), Vec3(size[0],0,0), color, thickness)
            self.drawLine(Vec3(0,0,0), Vec3(0,size[0],0), color, thickness)
            self.drawLine(Vec3(0,0,0), Vec3(0,0,size[0]), color, thickness)
        else:
            # The up, forward and left directions in the np's local space could be
            # computed using it's transformation matrix as is done to move the
            # camera in class Camera.
            pass

    def drawRectangle(self, bottomleft, topright,color=None,thickness=None):
        """Draw the rectangle defined by bottomleft and topright (Point3)."""
        
        bl_x,bl_y,bl_z = bottomleft.getX(),bottomleft.getY(),bottomleft.getZ()
        tr_x,tr_y,tr_z = topright.getX(),topright.getY(),topright.getZ()

        self.drawLine(Vec3(bl_x,bl_y,bl_z), Vec3(tr_x,bl_y,tr_z), color, thickness)
        self.drawLine(Vec3(tr_x,bl_y,tr_z), Vec3(tr_x,tr_y,tr_z), color, thickness)
        self.drawLine(Vec3(tr_x,tr_y,tr_z), Vec3(bl_x,tr_y,bl_z), color, thickness)
        self.drawLine(Vec3(bl_x,tr_y,bl_z), Vec3(bl_x,bl_y,bl_z), color, thickness)
       
    def drawXYGrid(self, bottomleft, numSquares=10, squareSize=10, color=None,
                   thickness=None):
        """Draw a numSquares*numSquares grid of squares on the XZ plane, each
        square being of size sqaureSize*squareSize. Start drawing at position
        bottomleft (Vec2), and move in the +X and +Y directions."""
        
        for i in range(0, numSquares):
            for j in range(0, numSquares):
                squarebottomleft = Vec3(bottomleft.getX()+(i*squareSize),
                                        bottomleft.getY()+(j*squareSize),0)
                squaretopright = Vec3(squarebottomleft.getX()+squareSize,
                                      squarebottomleft.getY()+squareSize,0)
                self.drawRectangle(squarebottomleft,squaretopright,color,
                                     thickness)

    def drawCuboid(self, bottomleft=None, topright=None, color=None,
                   thickness=None):
        """Draw the edges of a cuboid between the opposite corners bottomleft
        and topright, and with the given color and line thickness."""
    
        if bottomleft is None: bottomleft = Point3(0,0,0)
        if topright is None: topright = bottomleft + Vec3(100,100,100)
                
        bl_x,bl_y,bl_z = bottomleft.getX(),bottomleft.getY(),bottomleft.getZ()
        tr_x,tr_y,tr_z = topright.getX(),topright.getY(),topright.getZ()

        # Draw the bottom rectangle.        
        self.drawRectangle(Vec3(bl_x,bl_y,bl_z), Vec3(tr_x,tr_y,bl_z), color,
                           thickness)

        # Draw the top rectangle.
        self.drawRectangle(Vec3(bl_x,bl_y,tr_z), Vec3(tr_x,tr_y,tr_z), color,
                           thickness)
                
        # Draw the four lines connecting the two rectangles at each corner.
        self.drawLine(Vec3(bl_x,bl_y,bl_z), Vec3(bl_x,bl_y,tr_z), color, thickness)
        self.drawLine(Vec3(tr_x,bl_y,bl_z), Vec3(tr_x,bl_y,tr_z), color, thickness)
        self.drawLine(Vec3(tr_x,tr_y,bl_z), Vec3(tr_x,tr_y,tr_z), color, thickness)
        self.drawLine(Vec3(bl_x,tr_y,bl_z), Vec3(bl_x,tr_y,tr_z), color, thickness)


    def drawXYCircle(self,radius=1,angle=360,pos=None,color=None,
                     thickness=None):
        """Draw a circle (if angle = 360) or arc (if angle < 360) on the XZ
        plane at position pos (Vec3)."""            

        if pos is None: pos = Vec3(0,0,0)
        if angle>360: angle=360
        
        if color is None: color = (1,1,1,1)
        if thickness is None: thickness = 1
                
        self.setColor(*color)
        self.setThickness(thickness)        

        angleRadians = deg2Rad(angle)
        numSteps = 100        
        y = math.sin(0) * radius
        x = math.cos(0) * radius
        self.moveTo(pos + Point3(x,y,0))       
        for i in range(1,numSteps + 1):
            a = angleRadians * i / numSteps
            y = math.sin(a) * radius
            x = math.cos(a) * radius
            self.drawTo(pos + Point3(x,y,0))

    def drawXYReticle(self,radius=1,angle=360,pos=None,color=None,
                     thickness=None):
        """Draw a reticle (circle and cross) on the XZ plance at position pos
        (Vec3)."""
                     
        self.drawXYCircle(radius,angle,pos,color,thickness)
        self.drawLine(pos,pos+Vec3(radius,0,0),color,thickness)
        self.drawLine(pos,pos+Vec3(0,radius,0),color,thickness)
        self.drawLine(pos,pos+Vec3(-radius,0,0),color,thickness)
        self.drawLine(pos,pos+Vec3(0,-radius,0),color,thickness)

    def drawSphere(self,radius=1,center=None):
        """Draw a sphere of the given radius and centred at the given point
        (Point3). If center is None draw the sphere at the origin.
        It'd be cool to be able to draw a sphere. I don't know how though.
        """    
        pass

if __name__ == "__main__":
    """Run a test environment to demonstrate usage of (and debug) the above
    draw functions."""
    
    from direct.showbase.DirectObject import DirectObject
    from direct.task import Task
    from direct.gui.DirectGui import DirectLabel
    # Note: we don't bother to import DirectStart here to start the Panda3D
    # engine. It is imported by camera.py.
    from camera import Camera
    import random

    class Vehicle:
        """A moving 'vehicle' represented by a circle and with a floating text
        label that changes over time.
        
        Each vehicle uses its own Draw object, and moves itself around by
        moving the NodePath to which the Draw object is attached. A DirectLabel
        is attached to the same NodePath for textual annotation."""
    
        def __init__(self):
            """Initialise the vehicle."""
            
            # Draw a circle to represent this vehicle.
            d = Draw()                        
            d.drawXYCircle(pos = Vec3(0,0,0))
            node = d.create()
            self.np = NodePath(node)   
            self.np.reparentTo(render)
            
            # Add a text label above and to the side of this vehicle.  
            self.label=DirectLabel( parent=self.np,
                                    text="Hello! :)", 
                                    text_wordwrap=10,
                                    relief=None,
                                    text_scale=(0.5,0.5),
                                    text_frame=(0,0,0,0),
                                    text_bg=(0,0,0,0),
                                    color=(0.88,0,0.88,1))
            self.label.setPos(2,4,2) # Offset text a little from vehicle.
            self.label.setBillboardAxis()                   
                                
            # Compute a random direction for this vehicle to move in.                    
            self.direction = Vec3(random.random()-0.5,random.random()-0.5,0)
            self.direction.normalize()

            # Add a task to move this vehicle around.
            self.prevtime = 0
            taskMgr.add(self.move,"moveTask")        
            
        def move(self,task):
            """Move the vehicle around by moving it's NodePath."""
            
            speed = 5
            elapsed = task.time - self.prevtime

            # A hard-coded ugly hack to make the vehicle bounce around inside
            # the grid.
            self.np.setPos(self.np.getPos()+((self.direction*speed)*elapsed))
            text = 'X: ' + str(self.np.getPos().getX()) + '\n'           
            text += 'Y: ' + str(self.np.getPos().getY()) + '\n'
            self.label['text'] = text
                                        
            if (self.np.getPos().getX() < -50 or
                self.np.getPos().getX() > 50 or 
                self.np.getPos().getY() < -50 or
                self.np.getPos().getY() > 50
                ):
                    self.direction = -self.direction
                        
            self.prevtime = task.time
            return Task.cont
        
    class World(DirectObject):
        """The test environment."""
    
        def __init__(self):            
            """Initialise the test environment."""
            
            # Set a black background
            base.win.setClearColor(Vec4(0,0,0,1))
            
            # Setup a camera.
            self.camera = Camera(Vec3(50,50,50))
            self.camera.lookAt(0,0,0)       
            
            # Accept some keys to move the camera.
            self.accept("a", self.camera.setControl, ["left",1])
            self.accept("a-up", self.camera.setControl, ["left",0])
            self.accept("d", self.camera.setControl, ["right",1])
            self.accept("d-up", self.camera.setControl, ["right",0])
            self.accept("w", self.camera.setControl, ["up",1])
            self.accept("w-up", self.camera.setControl, ["up",0])
            self.accept("s", self.camera.setControl, ["down",1])
            self.accept("s-up", self.camera.setControl, ["down",0])
            self.accept("arrow_up", self.camera.setControl, ["forward",1])
            self.accept("arrow_up-up", self.camera.setControl, ["forward",0])
            self.accept("arrow_down", self.camera.setControl, ["backward",1])
            self.accept("arrow_down-up", self.camera.setControl, ["backward",0])
            self.accept("arrow_left", self.camera.setControl, ["strafe-left",1])
            self.accept("arrow_left-up", self.camera.setControl, ["strafe-left",0])
            self.accept("arrow_right", self.camera.setControl, ["strafe-right",1])
            self.accept("arrow_right-up", self.camera.setControl,["strafe-right",0])
            
            # Accept the Esc key to exit.
            self.accept("escape", sys.exit)
             
            # Create the static elements of the test environment. Use one Draw
            # object for all the static elements.
            d = Draw() 

            # Make a red circle at the origin.     
            d.drawXYCircle(color=(0.667,0.33,0,1))       
            
            # Draw the global axes at the origin, using thick red lines.
            d.drawAxes(size=Vec3(5,5,5),color=(0.667,0.33,0,1),thickness=2)
                
            # Make a 100x100 grid centered at the origin, in grey.    
            d.drawXYGrid(Vec2(-50,-50),color=(0.2,0.2,0.2,1))
            
            # Draw a grey 100x100x100 cuboid, with the grid we previously drew
            # as the floor of the cuboid.                        
            d.drawCuboid(Vec3(-50,-50,0), Vec3(50,50,25), 
                         color=(0.2,0.2,0.2,1))
            
            # Add some rectangles and some circles dotted about, in purple.
            random.seed()
            for i in range(0,4):
                bl_x = (random.random()-0.5)*100
                bl_y = (random.random()-0.5)*100
                tr_x = (random.random()-0.5)*100
                tr_y = (random.random()-0.5)*100
                d.drawRectangle(Point3(bl_x,bl_y,0),Point3(tr_x,tr_y,0),
                                color=(0.66,0,0.66,1))
                center_x = (random.random()-0.5)*100
                center_y = (random.random()-0.5)*100
                radius = random.random()*5
                d.drawXYCircle(radius=radius, pos=Point3(center_x,center_y,0),
                                          color=(0.66,0,0.66,1))
            
            node = d.create() # A special GeomNode that draws the shapes.
            np = NodePath(node)   
            np.reparentTo(render)
            
            # Create some moving vehicles.
            for i in range(0,10):
                vehicle = Vehicle()
                        
    w = World()
    run()
