"""
camera.py -- A camera for Panda3D that can be rotated or moved around by
setting values in a control map.

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
# TODO: add a camera with a state (FSM) and controls for switching between
#  states, where each state is a camera mode (floating camera,
#  following camera, etc
from direct.task import Task
from direct.showbase.DirectObject import DirectObject
import pandac.PandaModules as P
from direct.showbase.PythonUtil import clampScalar
from direct.interval import LerpInterval as LERP

CameraM = P.BitMask32.bit(3)#What shouldn't be between camera and avatar.

def fromCol(parent,handler,type,mask = P.BitMask32.allOn()):
    """Setup a from collision solid.

    Last I checked CollisionPolygon 's and CollisionTube 's can't be used
    as from solids. If you pass one, it won't hit anything"""
    nodepath = parent.attachNewNode(P.CollisionNode('frmcol'))
    nodepath.node().addSolid(type)#add the solid to the new collisionNode
    nodepath.node().setFromCollideMask(mask)#allow selective masking
    nodepath.setCollideMask(P.BitMask32.allOff())#it's a from solid only.
    ####uncomment this line to make the collision solid visible:
    ##nodepath.show()
    base.cTrav.addCollider(nodepath,handler)#add to the traverser
    try:#the next line doesn't work on queues. (not necessary)
      handler.addCollider(nodepath,parent)#keep the ward out of trouble
    except:
      pass#Don't care. This method needs to work on queues too.
    return nodepath#we might need the new CollisionNode again later.

class Camera:
  """A camera with that can be rotated and moved around by setting values in
  a control map.
  """

  def __init__(self,pos):
    """Initialise the camera. Pos should be a Point3, the initial position
    of the camera."""

    self.prevtime = 0

    # The camera's controls:
    # 0 = off, 1 = on
    self.controlMap = {"left":0, "right":0, "up":0, "down":0, "forward":0,
               "backward":0, "strafe-left":0, "strafe-right":0}

    # Task that moves the camera according to the control map.
    taskMgr.add(self.move,"cameraMoveTask")

    base.disableMouse()
    base.camera.setPos(pos)

  def lookAt(self,point):
    """Wrapper method for base.camera.lookAt"""
    base.camera.lookAt(point)

  def lookAt(self,x,y,z):
    """Wrapper method for base.camera.lookAt"""
    base.camera.lookAt(x,y,z)

  def move(self,task):
    """Update the camera's position before rendering the next frame.

    This is a task function and is called each frame by Panda3D.

    Arguments:
    task -- A direct.task.Task object passed to this function by Panda3D.

    Return:
    Task.cont -- To tell Panda3D to call this task function again next
           frame.

    """
    elapsed = task.time - self.prevtime

    # Rotate the camera according to which of it's controls are activated.
    speed = 60
    if (self.controlMap["left"]!=0):
      base.camera.setH(base.camera.getH() + (elapsed*speed))
    if (self.controlMap["right"]!=0):
      base.camera.setH(base.camera.getH() - (elapsed*speed))
    if (self.controlMap["up"]!=0):
      base.camera.setP(base.camera.getP() - (elapsed*speed))
    if (self.controlMap["down"]!=0):
      base.camera.setP(base.camera.getP() + (elapsed*speed))
    if (self.controlMap["forward"]!=0 or self.controlMap["backward"]!=0):
      # This two-line Matrix trick is taken from the Roaming Ralph demo
      # that comes with Panda3D, see
      # <http://panda3d.org/phpbb2/viewtopic.php?t=1446> for an
      # explanation.
      forward = base.camera.getNetTransform().getMat().getRow3(1)
      forward.normalize()
      if (self.controlMap["forward"]!=0):
        base.camera.setPos(base.camera.getPos() + forward*(elapsed*speed))
      if (self.controlMap["backward"]!=0):
        base.camera.setPos(base.camera.getPos() - forward*(elapsed*speed))
    if (self.controlMap["strafe-left"]!=0 or self.controlMap["strafe-right"]!=0):
      # Compute the camera's right direction in the same way we compute
      # the forward direction above, using the NodePath's transformation
      # matrix.
      right = base.camera.getNetTransform().getMat().getRow3(0)
      right.setZ(0)
      right.normalize()
      if (self.controlMap["strafe-left"]!=0):
        base.camera.setPos(base.camera.getPos() - right*(elapsed*speed))
      if (self.controlMap["strafe-right"]!=0):
        base.camera.setPos(base.camera.getPos() + right*(elapsed*speed))

    # Store the task time and continue.
    self.prevtime = task.time
    return Task.cont

  def setControl(self, control, value):
    """Set the state of one of the camera's movement controls.

    Arguments:
    See self.controlMap in __init__.
    control -- The control to be set, must be a string matching one of
           the strings in self.controlMap.
    value -- The value to set the control to.

    """

    self.controlMap[control] = value

class EdgeScreenTracker(DirectObject):
  """Mouse camera control interface."""
  def __init__(self,avatar,offset=P.Point3.zero(), dist=10,
      rot=20,zoom=(30,400),pitch=(-15,0)):
    # Disable default camera interface.
    base.disableMouse()
    # Set parameters
    self.zoomLvl = dist #camera starting distance
    self.speed = 1.0/rot # Controls speed of camera rotation.
    self.zoomClamp=zoom#clamp zoom in this range
    self.clampP=pitch#clamp pitch in this range
    self.target = avatar.attachNewNode('camera target')
    self.target.setPos(offset)#offset target from avatar.
    #Load the camera
    self.__loadCamera()
    #Enable new camera interface
    self.accept('arrow_up', self.cameraZoom,[0.7])#Translated. For zooming.
    self.accept('arrow_down', self.cameraZoom,[1.3])
    self.accept('arrow_left', self.rotateCam,[P.Point2(-10,0)])
    self.accept('arrow_right', self.rotateCam,[P.Point2(10,0)])
    taskMgr.add(self.mousecamTask, "mousecamTask")#For edge screen tracking
  def __loadCamera(self):
    """Only seperate for organisation, treat it as is part of __init__() .

    Load the camera & setup segmet & queue for detecting obstructions."""
    #Don't rotate the target with the avatar.
    self.target.node().setEffect(P.CompassEffect.make(render))
    camera.reparentTo(self.target)# Attach the camera to target.
    camera.setPos(0,-self.zoomLvl,50)# Position the camera
    self.rotateCam(P.Point2(0,0))# Initialize gimbal clamps.
    self.Q = P.CollisionHandlerQueue()# New queue for camera.
    self.segment = fromCol(self.target,self.Q,
      P.CollisionSegment(P.Point3.zero(),camera.getPos(self.target)),
      P.BitMask32(CameraM))#CameraM into segment between camera & target.
  def mousecamTask(self,task):
    """Rotate camera when the pointer moves to the edges of the screen.

    Also temporarily zooms in past an obstructing CameraM'ed object."""
    self.setDist(self.zoomLvl)#preset dist to current zoom level.
    if self.Q.getNumEntries() > 0:#if there was a collision
      self.Q.sortEntries() #so we get the closest collision to avatar
      point=self.Q.getEntry(0).getSurfacePoint(self.target)#get the point
      if point.lengthSquared()<camera.getPos().lengthSquared():#not out.
        self.setDist(point.length())#Temporarily zoom to point.
    camera.lookAt(self.target)# always point camera at target
    if not base.mouseWatcherNode.hasMouse():#See if the mouse is available.
      return Task.cont#if no, just loop again.
    # Get the relative mouse position, its always between 1 and -1
    mpos = base.mouseWatcherNode.getMouse()
    if mpos.getX() > 0.99:
      self.rotateCam(P.Point2(-10,0))
    elif mpos.getX() < -0.99:
      self.rotateCam(P.Point2(10,0))
    if mpos.getY() > 0.9:
        self.rotateCam(P.Point2(0,-3))
    elif mpos.getY() < -0.9:
      self.rotateCam(P.Point2(0,3))
    return Task.cont#loop again.
  def rotateCam(self,arc):
    """Setup a lerp interval to rotate the camera about the target."""
    newP=clampScalar(self.target.getP()-arc.getY(),*self.clampP)#Clamped.
    newH=self.target.getH()+arc.getX()#Not clamped, just added.
    LERP.LerpHprInterval(self.target, self.speed,#Setup the interval\
      P.Vec3(newH,newP,self.target.getR(), ), ).start()#and start it.
  def cameraZoom(self,zoomFactor,):
    """Scale and clamp zoom level, then set distance by it."""
    self.zoomLvl=clampScalar(self.zoomLvl*zoomFactor,*self.zoomClamp)
    self.setDist(self.zoomLvl)
  def setDist(self,newDist):
    """Set camera distance from the target."""
    vec = camera.getPos()
    vec.normalize()#set length to 1
    vec*=newDist#set length to clamped value
    camera.setFluidPos(vec)#move the camera to new distance
    #Move the segment end but keep it a little behind and below the camera.
    ##zuck
    ###self.segment.node().getSolid(0).setPointB(
      ###self.target.getRelativePoint(camera, P.Point3(0,-2,-1)))
    self.segment.node().modifySolid(0).setPointB(
      self.target.getRelativePoint(camera, P.Point3(0,-2,-1))
    )
#end EdgeScreenTracker
