"""
terrain.py

A fractal terrain model generator based on the diamond-square algorithm, with a
test/demo scene.

makeTerrain() returns two NodePath's: one to a renderable terrain model, and
one to collision geometry for the model.

A fractal heightmap is generated following the diamond-square algorithm, and
the terrain model is built from the heightmap.

Terrain models from makeTerrain() can be chained together (placed side-by-side)
to create large areas of terrain.

If this module is run directly then a test/demo scene with 4 chained terrain
models is loaded.

The diamond-square algorithm is a basic, 3D fractal terrain generation
algorithm, see: http://www.gameprogrammer.com/fractal.html

Pre-populating heightmaps before running the diamond-square algorithm to
produce (for example) a terrain block with one big mountain in the middle
should be possible (not implemented). After the algorithm paths through the
terrain can be flattened to create paths or riverbeds or areas can be flattened
to (for example) allow for placing of buildings (also not implemented, although
there is sample code for creating a path). Also after the algorithm smoothing
can be applied to the terrain to make it look more natural (implemented) and
the edges of the terrain can be flattened to make it chainable with other
terrains (implemented).

TODO:

* Apply some texture to the terrain. A simple grass texture repeated on each
  quad of the model would be okay. Fractally generated texture? Distance and
  detail textures?

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
# TODO: Get makeHeightMap to return a PNMImage and makeTerrainFromHeightmap to
# read a PNMImage. Then makeTerrainFromHeightmap could be run on Perlin noise
# from perlin.py. Could have makeDiamondSquareHeightmap and
# makePerlinHeightmap.
# Python imports
import random,os
# Panda imports
import pandac.PandaModules as P
from direct.task.Task import Task
from pandac.PandaModules import PNMImage
# Custom imports
# FIXME: Should collisionmasks and SphereObstacles be set here or in scene.py?
# scene.py could access the terrain collision model and list of trees through
# Terrain and set the collision masks, add SphereObstacles.
import character as C
from perlin import greenNoise
from vehicle import cTrav as vehicleCTrav
from vehicle import obstacleMASK,offMASK,obstacleHandler
from obstacles import SphereObstacle
# Initialise a CollisionTraverser for this module
cTrav = P.CollisionTraverser('Global CollisionTraverser of terrain.py')
# Setup a task to traverse the CollisionTraverser
def traverse(task):
  if len(taskMgr.getTasksNamed("Tree step task")) == 0:
    # FIXME: This assumes that all Terrain objects will be added at the
    # very beginning of the simulation. (Any that are added after this task
    # returns 'done' will not benefit from a collision traverser for
    # trees). Give each Terrain object its own collision traverser and task
    # which it can stop when it no longer needs it?
    return Task.done
  else:
    cTrav.traverse(render)
    return Task.cont
collisionTraverse=taskMgr.add(traverse,"collisionTraverse task of terrain.py")

##############################################################################
# HeightMap and terrain model utility functions (used by class Terrain below).
# These should probably become methods of terrain some day.

def smoothHeightMap(map,n=1):
  """Return a smoothed out copy of a given heightmap.

  keyword arguments:
  map -- a 2D list of numerical values.
  n -- I can do it n times!

  """
  newmap = list(map)
  for x in range(0,n):
    temp = list(newmap)
    for i in range(1,len(temp)-1):
      for j in range(1,len(temp)-1):
        tl = temp[i-1][j+1]
        t = temp[i][j+1]
        tr = temp[i+1][j+1]
        l = temp[i-1][j]
        c = temp[i][j]
        r = temp[i+1][j]
        bl = temp[i-1][j-1]
        b = temp[i][j-1]
        br = temp[i+1][j-1]
        newmap[i][j] = (tl+t+tr+l+(8*c)+r+bl+b+br)/16
  return newmap

def makeHeightMap(size=33,h=6,smooth=True):
  """Return a chainable size x sixe heightmap (2D list) computed using the
  diamond-square algorithm.

  A heightmap can be used (for example) to build terrain models, or sky,
  water or detail textures.

  Keyword arguments:
  size -- the size of the terrain heightmap in pixels (default 33).
      Acceptable values are 33,129,513
  h -- the 'hilliness' factor (default 6)
  smooth -- whether or not to smooth the heightmap after executing the
       diamond-square algorithm (default True)

  """
  # The number of times (n) the smoothHeightMap() is applied after generating
  # the initial heightmap depends on the initial value of h ('hillier'
  # heightmaps get smoothed out more than flat ones).
  if h <= 2: n = 0
  elif h <= 3: n = 2
  elif h <= 12: n = 3
  elif h <= 16: n = 4
  else: n = 5

  # 33, 129 and 513 are known to work...
  # Any number that is a power of 2 plus one should work, but not all do,
  # must be a bug in here somewhere.
  if size not in [33,129,513]: size = 33

  # Create a size x size 2D list of 0's
  heightmap = [[x]*size for x in [0]*size]

  # Fill in the heightmap using the diamond-square algorithm.
  side = len(heightmap)-1 # Length of the side of the squares
  while side > 0:
    # Perform the diamond step for each square.
    for i in range(0,len(heightmap)-1,side):
      for j in range(0,len(heightmap)-1,side):
        bl = heightmap[i][j]
        tr = heightmap[i+side][j]
        tl = heightmap[i+side][i+side]
        br = heightmap[i][i+side]
        height = ((bl+tr+tl+br)/4) + ((random.random()-0.5)*h)
        heightmap[i+side/2][j+side/2] = height
    # Perform the square step for each diamond.
    side = side/2
    for i in range(0,len(heightmap)-1,side):
      for j in range(0,len(heightmap)-1,side):
        l = heightmap[i][j+side/2]
        b = heightmap[i+side/2][j]
        t = heightmap[i+side/2][j+side]
        r = heightmap[i+side][j+side/2]
        height = ((l+b+t+r)/4) + ((random.random()-0.5)*h)
        heightmap[i+side/2][j+side/2] = height
    h = h/2
    side = side/2

  # It is possible to create 'paths' or 'rivers' through hilly terrain by
  # flattening out a path through the heightmap after executing the
  # diamond-square algorithm but before smoothing the heightmap. This
  # flattening creates a path or riverbed with very jagged edges which are
  # then smoothed with the surrounding terrain when smoothHeightMap is run.
  #
  # An interesting possibility to create curvy paths is to load a 'paths'
  # image file at this stage in which black pixels will be flattened and
  # white pixels will be ignored.
  #
  # You really need high-resolution hieghtmaps for this stuff to work well.
  #
  # This flattens a diagonal path through the middle of the heightmap:
  #from math import floor
  #width = 5
  #for i in range(1,len(heightmap)-1):
  #  for a in range(i-floor(width/2),i+floor(width/2)):
  #    for b in range(i-floor(width/2),i+floor(width/2)):
  #      heightmap[a][b] = -1

  # Flatten the edges of the heightmap to make it chainable with other
  # heightmaps produced by this same method.
  for i in range(0,len(heightmap)):
    heightmap[0][i] = 0
    heightmap[i][0] = 0
    heightmap[0][len(heightmap)-1] = 0
    heightmap[len(heightmap)-1][0] = 0

  if smooth: return smoothHeightMap(heightmap,n)
  else: return heightmap

def makeQuad(renderVP,collisionVP,bl=None,tr=None,br=None,tl=None):
  """Return a tuple of EggPolygons (renderQuad,collisionQuad): renderable and
  collision geometry for a 4-vertex polygon.

  Keyword arguments:
  renderVP -- EggVertexPool to which vertices will be added for the
        renderable quad
  collisionVP -- EggVertexPool to which vertices will be added for the
           collidable quad
  bl -- bottom-left vertex of quad (Point3D). Default (-10,-10,0)
  tr -- top-right vertex of quad (Point3D). Default (10,10,0)
  br -- bottom-right vertex of quad (Point3D)
  tl -- top-left vertex of quad (Point3D)
  If either of br or tl is None then sensible defaults will be computed
  according to the values of bl and tr.

  """
  if bl is None: bl = P.Point3D(-10,-10,0)
  if tr is None: tr = P.Point3D(10,10,0)
  if br is None:
    l = tr.getX() - bl.getX()
    br = bl + P.Vec3D(l,0,0)
  if tl is None:
    w = tr.getY() - bl.getY()
    tl = bl + P.Vec3D(0,w,0)
  renderQuad = P.EggPolygon()
  collisionQuad = P.EggPolygon()
  for corner in [bl,br,tr,tl]:
    vertex = P.EggVertex()
    vertex.setPos(corner)
    collisionVertex = P.EggVertex()
    collisionVertex.setPos(corner)
    renderQuad.addVertex(renderVP.addVertex(vertex))
    collisionQuad.addVertex(collisionVP.addVertex(collisionVertex))

  # A bug in Panda3D means that auto-triangulation of concave polygons
  # fails for some combinations of vertices (the polygon is not properly
  # transformed into triangles) creating gaps in a model where polygons are
  # missing. We can check for this failure by calling
  # EggPolygon.triangulateInto() and checking its return value. If False is
  # returned we reverse the vertex order of the polygon (so it now faces
  # downward instead of up) and reverse the backface flag so the downside
  # (which is now facing up) is rendered instead of the upside (which is now
  # facing down). The result looks more or less the same as if the polygon
  # had triangulated correctly in the first place (except the shading will be
  # different). Thanks to ynjh_jo for this workaround.
  egn=P.EggGroupNode() # triangulateInto() stores the resulting polygons in
             # an EggGroupNode, which we discard, we only care
             # about the True or False return value
  if not renderQuad.triangulateInto(egn,0):
     # Triangulation failed, fix it.
     renderQuad.reverseVertexOrdering()
     renderQuad.setBfaceFlag(1)
     collisionQuad.reverseVertexOrdering()
     collisionQuad.setBfaceFlag(1)

  renderQuad.recomputePolygonNormal() # Use faceted not smoothed lighting
  return renderQuad,collisionQuad

def makeTerrainFromHeightMap(heightmap):
  """Return a tuple of NodePaths (renderNodePath,collisionNodePath) to
  renderable and collision geometry for a chainable terrain model built from
  the given heightmap.

  collisionNodePath is parented to renderNodePath. renderNodePath is not
  parented to anything by this function.

  Every 3x3 group of quads in the collision geometry is collected under a
  PandaNode for efficient collision detection. This could be improved on by
  building both the renderable and collision geometry as octrees.

  Keyword arguments:
  heightmap -- a 2D list of height values.

  """
  size = len(heightmap)
  renderNodePath = P.NodePath('')
  renderEggData = P.EggData()
  renderVertexPool = P.EggVertexPool('')
  collisionNodePath = renderNodePath.attachNewNode('')
  # Supply the EggGroup & EggVertexPool for the first node.
  collisionVertexPool = P.EggVertexPool('')
  collisionEggGroup = P.EggGroup('')
  collisionEggGroup.addObjectType('barrier')
  # We group every (numQuadGrid x numQuadGrid) quads under 1 collision node.
  numQuadGrid=3
  # The modulo of (size-2)/numQuadGrid marks when the quads must be grouped
  # into 1 geom.
  edgeMod=(size-2)%numQuadGrid
  for i in range(0,len(heightmap)-1,numQuadGrid):
    # Limit nextIrange to avoid it from jump over the edge.
    nextIrange=min(numQuadGrid,len(heightmap)-1-i)
    for j in range(0,len(heightmap)-1):
      for nextI in range(0,nextIrange):
        bl = P.Point3D(i+nextI,j,heightmap[i+nextI][j])
        tr = P.Point3D(i+nextI+1,j+1,heightmap[i+nextI+1][j+1])
        br = P.Point3D(i+nextI+1,j,heightmap[i+nextI+1][j])
        tl = P.Point3D(i+nextI,j+1,heightmap[i+nextI][j+1])
        # Construct polygons (quads) with makeQuads.
        renderQuad,collisionQuad = makeQuad(renderVertexPool,
                          collisionVertexPool,
                          bl,tr,br,tl)
        renderEggData.addChild(renderQuad)
        collisionEggGroup.addChild(collisionQuad)
        if j%numQuadGrid==edgeMod and nextI==nextIrange-1:
           # Group the previous (numQuadGrid x numQuadGrid) quads under
           # collision node.
           collisionEggData = P.EggData()
           collisionEggData.addChild(collisionEggGroup)
           pandaNode = P.loadEggData(collisionEggData)
           nodePath = collisionNodePath.attachNewNode(
                      pandaNode.getChild(0).getChild(0))
           # Uncomment the next line to see the collision geom.
           #nodePath.show()
           # Supply the EggGroup & EggVertexPool for the next node.
           collisionEggGroup = P.EggGroup('')
           collisionEggGroup.addObjectType('barrier')
           collisionVertexPool = P.EggVertexPool('')
  pandaNode = P.loadEggData(renderEggData)
  renderNodePath.attachNewNode(pandaNode)
  return renderNodePath,collisionNodePath

def makeTerrain(size=33,h=6):
  """Return a tuple of NodePaths (renderNodePath,collisionNodePath), already
  attached into the Panda3D scene graph, to renderable and collision geometry
  for a chainable fractal terrain model built to the given size and hilliness
  values.

  See also makeHeightMap() and makeTerrainFromHeightMap().

  Keyword arguments:
  size -- the size (in vertices) of one side of the square terriain model
      (default 33) Note: only certain size values will be accepted, see
      makeHeightMap().
  h -- the 'hilliness' factor of the terrain (default 6)

  """

  heightmap = makeHeightMap(size,h)
  renderNodePath,collisionNodePath = makeTerrainFromHeightMap(heightmap)

  return renderNodePath,collisionNodePath

###############################################################################

models = {} # dict of {pathToModel:model}

def setRadiusTag(model):
  """Set a tag on the given model storing the models XY radius (ignoring how
  short or tall it is in the Z dimension).

  This radius tag can be retrieved later to (for example) create a collision
  sphere that will bound the model

  Arguments:
  model -- the model to tag

  """
  # Get the two points (in a list) that define the axis-aligned bounding
  # box of the scaled model.
  bounds=model.getTightBounds()
  # Now subtract the two, giving a vector from one to the other, the
  # length of which is the diameter of the aabb
  vector=(bounds[1]-bounds[0])
  # Take the greatest of the X and Y components of this vector, ignoring
  # the Z component, and halve it go get the X/Y radius of the aabb
  radius=max(vector[0],vector[1])*.5
  # Store the radius in a tag attached to the model
  model.setPythonTag('radius',radius)


def loadModel(path,parent):
  """Load a model from the file given by path, parent it to the given parent
  node and return the NodePath to the newly loaded model.

  Maintains a global dictionary of loaded models and if a model if called
  twice for the same model the model is instanced instead of being loaded
  again.

  """
  global models
  if models.has_key(path):
    model = models[path].copyTo(parent)
    scale = 0.75+random.random()/2
    model.setScale(scale)
    setRadiusTag(model)
  else:
    modelRoot = loader.loadModelCopy(path)
    # New models are loaded in a non-standard way to allow flattenStrong
    # to work effectively over them.
    model = P.NodePath('model')
    scale = 0.75+random.random()/2
    model.setScale(scale)
    modelRoot.getChildren().reparentTo(model)
    model.reparentTo(parent)
    setRadiusTag(model)
    models[path] = model
  return model

class Tree:

  def __init__(self,pos=None):
    """Initialise the tree."""

    # Models and CollisionSolids are parented to one prime NodePath
    if pos is None: pos = P.Vec3(0,0,0)
    self.pos = pos
    self.prime = P.NodePath('tree')
    self.prime.setPos(self.pos)

    dir = "models/trees" # FIXME: hardcoded models dir

    # Choose a random model from dir and load it.
    trees = [f for f in os.listdir(dir) if os.path.isfile(os.path.join(dir,f)) and f.endswith('.egg')]
    tree = random.choice(trees)
    self.np = loadModel(os.path.join(dir,tree),self.prime)

    # TODO: Give each tree a random orientation

    # Initialise the Tree's CollisionRay which is used with a
    # CollisionHandlerQueue to find the height of the terrain below the
    # tree and move the tree to that height (see self.step)
    self.raynp = self.prime.attachNewNode(P.CollisionNode('colNode'))
    self.raynp.node().addSolid(P.CollisionRay(0, 0, 3, 0, 0, -1))
    self.handler = P.CollisionHandlerQueue()
    cTrav.addCollider(self.raynp,self.handler)
    #self.raynp.show()
    # We only want our CollisionRay to collide with the collision
    # geometry of the terrain, se we set a mask here.
    self.raynp.node().setFromCollideMask(C.floorMASK)
    self.raynp.node().setIntoCollideMask(C.offMASK)

    # Add a task for this Tree to the global task manager.
    taskMgr.add(self.step,"Tree step task")

  def step(self,task):

    if self.handler.getNumEntries() > 0:
      self.handler.sortEntries()
      entry = self.handler.getEntry(0)
      point = entry.getSurfacePoint(render)
      self.prime.setPos(render,point)
      # Destroy the CollisionRay and related nodes and handlers, tell
      # Panda that this function doesn't need to be called anymore.
      cTrav.removeCollider(self.raynp)
      self.handler = None
      self.raynp.removeNode()
      return Task.done
    else:
      # No collision with the terrain was found, wiggle the tree a bit
      # (sometimes a tree's CollisionRay falls exactly through a tiny gap
      # in the terrain model and does not collide).
      self.prime.setX(self.prime.getX()+random.random())
      self.prime.setY(self.prime.getY()+random.random())
      return Task.cont

class Terrain:

  def __init__(self,pos=None,color=(0.6,0.8,0.5,1),scale=12,h=8,size=33,trees=0.7):

    # Initialise primary NodePath which everything else is parented to
    if pos is None: pos = P.Vec3(0,0,0)
    self.pos = pos
    self.prime = P.NodePath('Terrain primary NodePath')
    self.prime.setPos(self.pos)

    # Create terrain models (rendering and collision)
    self.tnp,self.collnp = makeTerrain(size=size,h=h)
    self.tnp.reparentTo(self.prime)
    self.tnp.setColor(*color)
    self.tnp.setScale(scale)
    # Shift the collision terrain a bit so rays from trees hit
    self.collnp.setPos(self.collnp,.0001,.0001,0)
    self.collnp.setCollideMask(C.floorMASK)

    # All trees in the scene are parented to one NodePath for flattening
    self.trees = self.prime.attachNewNode(P.PandaNode('trees'))
    self.treesColl = self.prime.attachNewNode(P.CollisionNode('treesColl'))
    self.treesColl.node().setIntoCollideMask(offMASK)
    self.treesColl.node().setFromCollideMask(obstacleMASK)
    #self.treesColl.show()
    vehicleCTrav.addCollider(self.treesColl,obstacleHandler)

    # Initialise trees
    img = greenNoise(imgSize=(size,size),scale=0.25)
    for x in range(0,size-1):
      for y in range(0,size-1):
        if img.getGreen(x,y) > trees:
          treepos = P.Vec3(x*scale+0.5*scale,y*scale+0.5*scale,50)
          tree = Tree(pos=treepos)
          tree.prime.reparentTo(self.trees)

    taskMgr.add(self.flatten,"Terrain flatten task")

  def flatten(self,task):

    if len(taskMgr.getTasksNamed("Tree step task")) == 0:
      for t in self.trees.getChildrenAsList():
        # Place a CollisionSphere over the model for obstacle avoidance
        pos=t.getPos()
        radius = t.getChild(0).getPythonTag('radius')
        radius*=1.2 # Make CollisionSphere 20% wider than the model
        s = SphereObstacle(pos[0],pos[1],pos[2]+radius,radius,show=True)
        s.cnp.reparentTo(self.treesColl)
      self.prime.flattenStrong()
      print 'FLATTENED'
      return Task.done
    else:
      return Task.cont


if __name__ == "__main__":

  # Python imports
  import math,sys
  # Panda3D imports
  import direct.directbase.DirectStart
  from direct.showbase.DirectObject import DirectObject

  # Custom imports
  from camera import Camera

  class TestScene(DirectObject):
    """A simple test/demo scene for this module."""

    def __init__(self):
      """Initialise the test scene."""

      # Show the framerate
      base.setFrameRateMeter(True)

      # Make 4 terrain nodepath objects with different hilliness values
      # and arrange them side-by-side in a 2x2 grid, giving a big terrain
      # with variable hilly and flat areas.

      color = (0.6,0.8,0.5,1) # Bright green-ish
      scale = 12

      t1 = Terrain(color=color,scale=scale,trees=0.7)
      t1.prime.reparentTo(render)
      t2 = Terrain(color=color,scale=scale,h=24,pos=P.Vec3(32*scale,0,0),trees=0.5)
      t2.prime.reparentTo(render)
      t3 = Terrain(color=color,scale=scale,h=16,pos=P.Vec3(32*scale,32*scale,0),trees=0.3)
      t3.prime.reparentTo(render)
      t4 = Terrain(color=color,scale=scale,h=2,pos=P.Vec3(0,32*scale,0),trees=0.9)
      t4.prime.reparentTo(render)

      # Setup a camera.
      base.disableMouse()
      self.camera = Camera(P.Vec3(0,0,100))
      self.camera.lookAt(320,320,0)

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

      # Setup lighting.
      self.alight = P.AmbientLight('alight')
      self.alight.setColor(P.VBase4(0.35, 0.35, 0.35, 1))
      self.alnp = render.attachNewNode(self.alight)
      render.setLight(self.alnp)

      self.dlight = P.DirectionalLight('dlight')
      self.dlight.setColor(P.VBase4(0.4, 0.4, 0.4, 1))
      self.dlnp = render.attachNewNode(self.dlight)
      self.dlnp.setHpr(45, -45, 0)
      render.setLight(self.dlnp)

      self.plight = P.PointLight('plight')
      self.plight.setColor(P.VBase4(0.8, 0.8, 0.5, 1))
      self.plnp = render.attachNewNode(self.plight)
      self.plnp.setPos(160, 160, 50)

      self.slight = P.Spotlight('slight')
      self.slight.setColor(P.VBase4(1, 1, 1, 1))
      lens = P.PerspectiveLens()
      self.slight.setLens(lens)
      self.slnp = render.attachNewNode(self.slight)
      self.slnp.setPos(-20, -20, 20)
      self.slnp.lookAt(50,50,0)

      # Setup some scene-wide exponential fog.
      colour = (0.5,0.8,0.8)
      self.expfog = P.Fog("Scene-wide exponential Fog object")
      self.expfog.setColor(*colour)
      self.expfog.setExpDensity(0.0005)
      render.setFog(self.expfog)
      base.setBackgroundColor(*colour)

    # Event listeners. TODO: Register these with some keyboard or GUI
    # events.
    def toggleWireframe(self,status):
      base.toggleWireframe()

    def toggleFog(self,status):
      if status:
        render.setFog(self.expfog)
      else:
        render.clearFog()

    def toggleAmbient(self,status):
      if status:
        render.setLight(self.alnp)
      else:
        render.clearLight(self.alnp)

    def toggleDirectional(self,status):
      if status:
        render.setLight(self.dlnp)
      else:
        render.clearLight(self.dlnp)

    def togglePoint(self,status):
      if status:
        render.setLight(self.plnp)
      else:
        render.clearLight(self.plnp)

    def toggleSpot(self,status):
      if status:
        render.setLight(self.slnp)
      else:
        render.clearLight(self.slnp)

  # Run the test scene.
  s = TestScene()
  run()
