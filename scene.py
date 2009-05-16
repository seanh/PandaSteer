"""
scene.py --

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
# Python imports
import math,sys,random,os
# Panda3D imports
from pandac.PandaModules import Vec2,Vec3,Vec3D,DirectionalLight,VBase4,PointLight
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import TransparencyAttrib
from direct.gui.DirectGui import *
from pandac.PandaModules import TextNode
from pandac.PandaModules import AmbientLight,Spotlight,PerspectiveLens,Fog
from pandac import PandaModules as P
from direct.task.Task import Task
# Custom imports
from camera import Camera,EdgeScreenTracker
from terrain import Terrain
from obstacles import SphereObstacle
from containers import ContainerSquare
from steerVec import SteerVec
import character as C

# CollisionTraverser for Plants
base.cTrav = P.CollisionTraverser('CollisionTraverser of scene.py')
plantNode = P.NodePath('plantNode') # All plants are parented to one node for flattening
plantNode.reparentTo(render)

class Sea:
	def __init__(self):
		sea = loader.loadModel("models/sea1.egg")
		sea.reparentTo(render)
		sea.setScale(2000,2000,100)

class Scene(DirectObject):

  def __init__(self):
    """Initialise the scene."""

    # Show the framerate
    base.setFrameRateMeter(True)

    # Initialise terrain:
    # Make 4 terrain nodepath objects with different hilliness values
    # and arrange them side-by-side in a 2x2 grid, giving a big terrain
    # with variable hilly and flat areas.

    color = (0.6,0.8,0.5,1) # Bright green-ish
    scale = 12
    height = 18  # FIXME: For now we are raising the terrain so it
           # floats above the sea to prevent lakes from
           # appearing (but you still get them sometimes)

    t1 = Terrain(color=color,scale=scale,trees=0.7,pos=P.Point3(0,0,height))
    t1.prime.reparentTo(render)
    t2 = Terrain(color=color,scale=scale,h=24,pos=P.Point3(32*scale,0,height),trees=0.5)
    t2.prime.reparentTo(render)
    t3 = Terrain(color=color,scale=scale,h=16,pos=P.Point3(32*scale,32*scale,height),trees=0.3)
    t3.prime.reparentTo(render)
    t4 = Terrain(color=color,scale=scale,h=2,pos=P.Point3(0,32*scale,height),trees=0.9)
    t4.prime.reparentTo(render)

    #tnp1.setPos(tnp1,-32,-32,terrainHeight)

    # Initialise sea
    sea = Sea()

    # Initialise skybox.
    self.box = loader.loadModel("models/skybox/space_sky_box.x")
    self.box.setScale(6)
    self.box.reparentTo(render)

    # Initialise characters
    self.characters = []
    self.player = C.Character(model='models/eve',run='models/eve-run',
                  walk='models/eve-walk')
    self.player.prime.setZ(100)
    self.player._pos = SteerVec(32*12+random.random()*100,32*12+random.random()*100)
    self.player.maxforce = 0.4
    self.player.maxspeed = 0.55
    EdgeScreenTracker(self.player.prime,dist=200) # Setup camera
    for i in range(0,11):
      self.characters.append(C.Character())
      self.characters[i].prime.setZ(100)
      self.characters[i].wander()
      self.characters[i].maxforce = 0.3
      self.characters[i].maxspeed = 0.2
      self.characters[i]._pos = SteerVec(32*12+random.random()*100,32*12+random.random()*100)

    C.setContainer(ContainerSquare(pos=SteerVec(32*12,32*12),radius=31*12))

    #C.toggleAnnotation()

    # Initialise keyboard controls.
    self.accept("c",C.toggleAnnotation)
    self.accept("escape", sys.exit)

    # Setup CollisionRay and CollisionHandlerQueue for mouse picking.
    self.pickerQ = P.CollisionHandlerQueue()
    self.picker = camera.attachNewNode(P.CollisionNode('Picker CollisionNode'))
    self.picker.node().addSolid(P.CollisionRay())
    # We want the picker ray to collide with the floor and nothing else.
    self.picker.node().setFromCollideMask(C.floorMASK)
    self.picker.setCollideMask(P.BitMask32.allOff())
    base.cTrav.addCollider(self.picker,self.pickerQ)
    try:
      handler.addCollider(self.picker,camera)
    except:
      pass
    self.accept('mouse1',self.onClick)

    # Set the far clipping plane to be far enough away that we can see the
    # skybox.
    base.camLens.setFar(10000)

    # Initialise lighting
    self.alight = AmbientLight('alight')
    self.alight.setColor(VBase4(0.35, 0.35, 0.35, 1))
    self.alnp = render.attachNewNode(self.alight)
    render.setLight(self.alnp)

    self.dlight = DirectionalLight('dlight')
    self.dlight.setColor(VBase4(0.4, 0.4, 0.4, 1))
    self.dlnp = render.attachNewNode(self.dlight)
    self.dlnp.setHpr(45, -45, 0)
    render.setLight(self.dlnp)

    self.plight = PointLight('plight')
    self.plight.setColor(VBase4(0.8, 0.8, 0.5, 1))
    self.plnp = render.attachNewNode(self.plight)
    self.plnp.setPos(160, 160, 50)

    self.slight = Spotlight('slight')
    self.slight.setColor(VBase4(1, 1, 1, 1))
    lens = PerspectiveLens()
    self.slight.setLens(lens)
    self.slnp = render.attachNewNode(self.slight)
    self.slnp.setPos(-20, -20, 20)
    self.slnp.lookAt(50,50,0)

    # Initialise some scene-wide exponential fog
    colour = (0.5,0.8,0.8)
    self.expfog = Fog("Scene-wide exponential Fog object")
    self.expfog.setColor(*colour)
    self.expfog.setExpDensity(0.0005)
    render.setFog(self.expfog)
    base.setBackgroundColor(*colour)

    # Add a task for this Plant to the global task manager.
    self.stepcount = 0
    taskMgr.add(self.step,"Plant step task")

  def step(self,task):
    if self.stepcount < 3:
      self.stepcount+=1
      return Task.cont
    else:
      plantNode.flattenStrong()
      print render.analyze()
      return Task.done

  def onClick(self):
    """Handle the mouse-click event."""

    mpos=base.mouseWatcherNode.getMouse()
    # Makes the ray's origin the camera and make the ray point to mpos
    ###self.picker.node().getSolid(0).setFromLens(
      ###base.camNode,mpos.getX(),mpos.getY())
    ##zuck
    self.picker.node().modifySolid(0).setFromLens(
      base.camNode,mpos.getX(),mpos.getY()
    )
    # We don't want to traverse now, so wait for panda to do it then move.
    taskMgr.doMethodLater(.02,self.setDestination,'setDest')

  def setDestination(self,task):
    """Helper method for onClick.

    Find the position in the 3D scene that was clicked and pass it to the
    click method of the currently active plugin.

    """
    if self.pickerQ.getNumEntries() > 0:
      self.pickerQ.sortEntries()
      self.point=self.pickerQ.getEntry(0).getSurfacePoint(render)
      self.player.arrive(SteerVec(self.point.getX(),self.point.getY()))

# Run the test scene
s = Scene()
run()
