"""
pandaSteer.py -- A plugin-based steering behaviours demo for Panda3D.

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
import os,math,sys,random,imp
# Panda3D imports
import direct.directbase.DirectStart
from pandac import PandaModules  as P
from direct.showbase.DirectObject import DirectObject
from direct.task.Task import Task
from pandac.PandaModules import loadPrcFileData
loadPrcFileData("", "interpolate-frames 1") # Smooth (interpolated) animations
# Custom imports
from terrain import Terrain
from camera import Camera
from obstacles import SphereObstacle
import character as C
import vehicle as V

from direct.gui.OnscreenText import OnscreenText

base.cTrav = P.CollisionTraverser('Global CollisionTraverser of pandaSteer.py')

class PandaSteer(DirectObject):
  def __init__(self):
    """Initialise the scene, controls and plugins."""

    # Show the framerate
    base.setFrameRateMeter(True)

    # Make some terrain and colour, scale and position it.
    color,scale = (0.6,0.8,0.5,1),3
    trees = 2 # Hack to make sure there are no trees
    t = Terrain(color=color,scale=scale,trees=2.0,pos=P.Point3(-50,-50,0),h=3)
    t.prime.reparentTo(render)

    # Setup a camera, position and orient it to look down at the terrain.
    base.disableMouse()
    self.camera = Camera(P.Vec3(-100,-100,100))
    self.camera.lookAt(0,0,0)

    # Setup ambient and directional lights.
    self.alight = P.AmbientLight('alight')
    self.alight.setColor(P.VBase4(0.35, 0.35, 0.35, 1))
    self.alnp = render.attachNewNode(self.alight)
    render.setLight(self.alnp)

    self.dlight = P.DirectionalLight('dlight')
    self.dlight.setColor(P.VBase4(0.4, 0.4, 0.4, 1))
    self.dlnp = render.attachNewNode(self.dlight)
    self.dlnp.setHpr(45, -45, 0)
    render.setLight(self.dlnp)

    # Setup some scene-wide exponential fog.
    colour = (0.5,0.8,0.8)
    self.expfog = P.Fog("Scene-wide exponential Fog object")
    self.expfog.setColor(*colour)
    self.expfog.setExpDensity(0.0005)
    render.setFog(self.expfog)
    base.setBackgroundColor(*colour)

    # Setup keyboard controls.
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
    # r key restarts the current simulation.
    self.accept("r",self.restart)
    # tab key goes to next demonstration
    self.accept("tab",self.next)
    # c toggles annotation
    self.accept("c",C.toggleAnnotation)
    # h toggles text
    self.accept("h",self.toggleText)

    # Lists to hold the Characters and Obstacles currently in the scene.
    self.characters = []
    self.obstacles = []

    # Find and initialise all demo plugins. Plugins are loaded in order of
    # filename.
    self.plugins = []
    self.plugin = 0
    for file in sorted(os.listdir('plugins')):
      if file.endswith('.py'):
        modulename = file[:-3]
        try:
          file, filename, description = imp.find_module(
            modulename, ['plugins'])
          pluginmodule = imp.load_module(modulename,
            file, filename, description)
          if hasattr(pluginmodule, 'Plugin'):
            print 'Instantiating plugin from %s' % pluginmodule
            try:
              self.plugins.append(pluginmodule.Plugin())
            except Exception, e:
              print 'Urk! %s' % e
          else:
            print 'No plugin in %s' % pluginmodule
        finally:
          if file:
            file.close()

    # Create OnscreenText object.
    self.textOn=True
    self.text=''
    self.ostext = OnscreenText(text=self.text,pos=(-1.3, 0.9),scale=0.05,
                   fg=(0,0,0,1),align=P.TextNode.ALeft)

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

    # Start the demo.
    self.restart()
    taskMgr.add(self.wrap,"Wrap task of pandaSteer.py")
  
  def onClick(self):
    """Handle the mouse-click event."""
  
    mpos=base.mouseWatcherNode.getMouse()
    # Makes the ray's origin the camera and make the ray point to mpos
    ##zucker
    ###self.picker.node().getSolid(0).setFromLens(
      ###base.camNode,mpos.getX(),mpos.getY())
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
      self.plugins[self.plugin].click(self.point)

  def wrap(self,task):
    """If the current plugin requests it, wrap Character's around if they
    leave the terrain."""

    # FIXME: hardcoded terrain size and position here.
    if self.plugins[self.plugin].wrap:
      for character in self.characters:
        if character.getspeed() > 0:
          if character._pos.getX()<-50: character._pos.setX(45)
          if character._pos.getX()>45: character._pos.setX(-50)
          if character._pos.getY()<-50: character._pos.setY(45)
          if character._pos.getY()>45: character._pos.setY(-50)
    return Task.cont

  def next(self):
    """Move onto the next plugin, or go back to the first."""

    self.plugin += 1
    if self.plugin >= len(self.plugins): self.plugin = 0
    self.restart()

  def restart(self):
    """Restart the demo environment, its characters, and the current
    plugin."""

    # Destroy the Character objects.
    for character in self.characters:
      character.destroy()
    self.characters = [] # Python will garbage-collect them now

    # Create new Character objects for the plugin.
    for i in range(0,self.plugins[self.plugin].numVehicles):
      character = C.Character(name='Ralph',#FIXME: Give each character a different name!
                  avoidObstacles=False,avoidVehicles=False)
      self.characters.append(character)

    self.plugins[self.plugin].vehicles = self.characters
    self.plugins[self.plugin].restart()

    # Destroy the Obstacle objects.
    for obstacle in self.obstacles:
      obstacle.destroy()
    self.obstacles = [] # Python will garbage-collect them now
  
    # Create new Obstacle objects for the plugin.
    if self.plugins[self.plugin].obstaclesOn == True:
      self.obstacles = [SphereObstacle(-20,20,0,15),
               SphereObstacle(20,10,0,5),
               SphereObstacle(10,20,0,3),
               SphereObstacle(20,-15,0,10),
               SphereObstacle(20,-35,0,3),
               SphereObstacle(-3,-35,0,10),
               SphereObstacle(-25,-10,0,7),
               SphereObstacle(-25,-30,0,2),
              ]
    C.setContainer(self.plugins[self.plugin].container)

    self.text = self.plugins[self.plugin].text + """

r: restart this demo
Tab: go to next demo
w: look down
s: look up
a: look left
d: look right
Arrow keys: move camera
   forward, backward, left right
c: annotation on/off
h: get rid of this annoying text
Esc: exit"""
    self.toggleText()
    self.toggleText() # :/
  
  def toggleText(self):
    if self.textOn:
      self.ostext.setText('h: get that helpful text back')
      self.textOn=False
    else:
      self.ostext.setText(self.text)
      self.textOn=True

# Run the test scene.
p = PandaSteer()
run()
