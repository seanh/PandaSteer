"""
vehicle.py -- an abstract vehicle that moves over uneven terrain with various
        steering behaviors.

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
import sys
from math import fabs,sqrt
import random
# Panda imports
from pandac.PandaModules import *
from direct.task import Task
from direct.gui.DirectLabel import DirectLabel
# Custom imports
from steerVec import SteerVec

# Initialise CollisionTraverser for all vehicle-related collision detection
cTrav = CollisionTraverser('Global CollisionTraverser of vehicle.py')
# Setup a task to traverse the CollisionTraverser
def traverse(task):
  cTrav.traverse(render)
  return Task.cont
collisionTraverse=taskMgr.add(traverse,"collisionTraverse task of vehicle.py")
# CollisionHandler used for Vehicles avoiding static obstacles
obstacleHandler = CollisionHandlerQueue()
# CollisionHandler used to detect Vehicles colliding with eachother
collisionHandler = CollisionHandlerQueue()
floorMASK = BitMask32.bit(1) # Collide with floor only
obstacleMASK = BitMask32.bit(2) # Collide with static obstacles only
vehicleMASK = BitMask32.bit(3) # Collide with vehicles
offMASK = BitMask32.allOff() # Collide with nothing

container = None # Global vehicle container, all Vehicles will remain within
         # this container.
def setContainer(newContainer):
  # FIXME: check that newContainer implements the container protocol
  global container
  container = newContainer
def unsetContainer():
  global container
  container = None

vehicles = [] # Global list of Vehicles, used by Vehicles to avoid collisions
obstacles = []

show = False # Toggle visualiation & annotation of Vehicles on/off
def toggleShow():
  global show
  if show:
    cTrav.hideCollisions()
    show = False
  else:
    cTrav.showCollisions(render)
    show = True

class Vehicle:
  """A steerable point mass with various steering behaviors."""

  # FIXME: One fix that is needed is that wanderSide and wanderUp should be
  # reset whenever a character switches back to the wander behaviour. This
  # might prevent wandering characters from 'hugging' obstacles and
  # containers when wandering is combined with obstacle avoidance and/or
  # containment. What happens is that the characters wanders into the
  # obstacle, avoidance takes over and steers the character away from it,
  # then when wandering gets control again it picks up where it left off --
  # steering toward the obstacle.
  #
  # So make Vehicle a state-machine, with the state being the currently
  # active steering behaviour, so that there can be transition-in and
  # transition-out functions for steering behaviours?

  def __init__(self,pos=None,mass=1,maxforce=0.1,maxspeed=1,radius=1,
         avoidVehicles=True,avoidObstacles=True,container=None):
    """Initialise the vehicle."""

    self.mass = mass # mass = 1 means that acceleration = force
    self.maxforce = maxforce
    self.maxspeed = maxspeed
    self.radius = radius
    self.avoidVehicles = avoidVehicles # Avoid colliding with other Vehicles
    self.avoidObstacles = avoidObstacles # Avoid colliding with obstacles
    if pos is None: pos = Point3(0,0,0)
    # The Vehicle controls a primary NodePath to which other NodePath's for
    # CollisionSolids are parented. Other nodes, such as animated Actors,
    # can be attached to this NodePath by user classes.
    self.prime = NodePath('Vehicle primary NodePath')
    self.prime.reparentTo(render)
    self.prime.setPos(pos)
    # If the Vehicle finds itself outside of its container it will turn
    # back toward the container. The vehicle also stays within the global
    # container at all times. The idea is that Vehicle.container can be
    # used to restrict one particular vehicle to a space smaller than the
    # global container.
    self.container = container

    self._pos = SteerVec(pos.getX(),pos.getY()) # 2D pos for steering
                          # calculations

    self._velocity = SteerVec(0,0)
    self._steeringforce = SteerVec(0,0)
    self._steeringbehavior = []

    # Some steering behaviors make use of self._target, which can be
    # another Vehicle or a point in 2-space (SteerVec), or it might be None
    # indicating that no target is currently in use.
    self._target = None

    # Initialise the Vehicle's CollisionRay which is used with a
    # CollisionHandlerFloor to keep the Vehicle on the ground.
    self.raynp = self.prime.attachNewNode(CollisionNode('colNode'))
    self.raynp.node().addSolid(CollisionRay(0,0,3,0,0,-1))
    self.floorhandler = CollisionHandlerFloor()
    self.floorhandler.addCollider(self.raynp,self.prime)
    cTrav.addCollider(self.raynp,self.floorhandler)
    # Uncomment this line to show the CollisionRay:
    #self.raynp.show()

    # We only want our CollisionRay to collide with the collision
    # geometry of the terrain only, se we set a mask here.
    self.raynp.node().setFromCollideMask(floorMASK)
    self.raynp.node().setIntoCollideMask(offMASK)

    # Initialise CollisionTube for detecting oncoming obstacle collisions.
    x,y,z = self.prime.getX(),self.prime.getY(),self.prime.getZ()+3 #FIXME: Don't hardcode how high the tube goes, add height parameter to __init__ for both tube and sphere
    vx,vy = self._velocity.getX(),self._velocity.getY()
    r = self.radius
    f = self.prime.getNetTransform().getMat().getRow3(1)
    s = 10
    self.tube = CollisionTube(x,y,z,x+f.getX()*s,y+f.getY()*s,z,self.radius)
    self.tubenp = self.prime.attachNewNode(CollisionNode('colNode'))
    self.tubenp.node().addSolid(self.tube)
    # The tube should only collide with obstacles (and is an 'into' object)
    self.tubenp.node().setFromCollideMask(offMASK)
    self.tubenp.node().setIntoCollideMask(obstacleMASK)
    # Uncomment this line to show the CollisionTube
    #self.tubenp.show()

    # CollisionSphere for detecting when we've actuallyt collided with
    # something.
    self.sphere = CollisionSphere(x,y,z,self.radius)
    self.spherenp = self.prime.attachNewNode(CollisionNode('cnode'))
    self.spherenp.node().addSolid(self.sphere)
    # Only collide with the CollisionSphere's of other vehicles.
    self.spherenp.node().setFromCollideMask(obstacleMASK) # So the spheres of vehicles will collide with eachother
    self.spherenp.node().setIntoCollideMask(obstacleMASK) # So obstacles will collide into spheres of vehicles
    cTrav.addCollider(self.spherenp,collisionHandler)
    # Uncomment this line to show the CollisionSphere
    #self.spherenp.show()

    # Add a task for this Vehicle to the global task manager.
    self._prevtime = 0
    self.stepTask=taskMgr.add(self._step,"Vehicle step task")

    # Add the Vehicle to the global list of Vehicles
    vehicles.append(self)

    # Initialise the DirectLabel used when annotating the Vehicle
    # FIXME: Needs to be destroyed in self.destroy
    self.text = "no steering"
    self.label = DirectLabel(parent=self.prime,pos=(4,4,4),text=self.text,
                 text_wordwrap=10,relief=None,
                 text_scale=(1.5,1.5),text_frame=(0,0,0,1),
                 text_bg=(1,1,1,1))
    self.label.component('text0').textNode.setCardDecal(1)
    self.label.setBillboardAxis()
    self.label.hide()
    self.label.setLightOff(1)

    # A second label for speaking ('Ouch!', 'Sorry!' etc.)
    self.callout = DirectLabel(parent=self.prime,pos=(4,4,4),text='',
                 text_wordwrap=10,relief=None,
                 text_scale=(1.5,1.5),text_frame=(0,0,0,1),
                 text_bg=(1,1,1,1))
    self.callout.component('text0').textNode.setCardDecal(1)
    self.callout.setBillboardAxis()
    self.callout.hide()
    self.callout.setLightOff(1)

  def destroy(self):
    """Prepare this Vehicle to be garbage-collected by Python:

    Remove all of the Vehicle's nodes from the scene graph, remove its
    CollisionSolids from the global CollisionTraverser, clear its
    CollisionHandler, remove tasks

    After executing this method, any remaining references to the Vehicle
    object can be destroyed by the user module, and the Vehicle will be
    garbage-collected by Python.

    """
    cTrav.removeCollider(self.raynp)
    cTrav.removeCollider(self.spherenp)
    self.floorhandler.clearColliders()
    self.floorhandler = None
    self.prime.removeNode()
    taskMgr.remove(self.stepTask)
    vehicles.remove(self)

  def getspeed(self):
    """Return the speed (not the velocity) of this vehicle (float)."""

    return self._velocity.length()

  def getX(self):
    """Convenience method for accessing self._pos.getX()"""

    return self._pos.getX()

  def getY(self):
    """Convenience method for accessing self._pos.getY()"""

    return self._pos.getY()

  def getForward(self):
    """Return this vehicle's forward direction, a unit vector (SteerVec).

    """
    forward = SteerVec(self._velocity.getX(),self._velocity.getY())
    forward.normalize()
    return forward

  def getRight(self):
    """Return this vehicle's right direction, a unit vector (SteerVec).

    """
    right = self._velocity.rotate(90)
    right.normalize()
    return right

  def predictFuturePosition(self, predictionTime):
    """
    Return the predicted position (SteerVec) of this vehicle
    predictionTime units in the future using a simple linear predictor.

    """
    return self._pos + (self._velocity * predictionTime)

  # Public methods used to activate steering behaviors. Once a steering
  # behavior is activated it will be applied every simulation frame until
  # another steering behavior is activated. Only one steering behavior can
  # be active at once.

  def stop(self):
    """Activate 'no steering' behavior. No steering force will be applied.
    """

    self._steeringbehavior = []

  def seek(self, target):
    """Activate seek steering behavior. target should be a SteerVec.
    """

    self._steeringbehavior = ['Seek']
    self._target = target

  def flee(self, target):
    """Activate flee steering behavior. target should be a SteerVec.
    """

    self._steeringbehavior = ['Flee']
    self._target = target

  def pursue(self, target):
    """Activate pursue steering behavior. target should be another Vehicle.
    """

    self._steeringbehavior = ['Pursue']
    self._target = target

  def follow(self, target):
    """Activate follow steering behavior. target should be another Vehicle.
    """

    self._steeringbehavior = ['Follow']
    self._target = target

  def evade(self, target):
    """Activate evade steering behavior. target should be another Vehicle.
    """

    self._steeringbehavior = ['Evade']
    self._target = target

  def arrive(self, target):
    """Activate the arrive steering behavior. target should be a SteerVec.
    """

    self._steeringbehavior = ['Arrive']
    self._target = target

  def wander(self):
    """Activate wander steering behavior.
    """

    # FIXME: when mixin behaviours are in use, wanderSide and wanderUp
    # should be reset every time a mixin is activated.
    self.wanderSide=0
    self.wanderUp=0
    self._steeringbehavior = ['Wander']

  def followPath(self,path,loop=False):
    self._steeringbehavior = ['FollowPath']
    self._path = path
    self._waypoint = path.getNearestWaypoint(self._pos)
    self._loop = loop

  # Private methods to compute the steering vectors for steering behaviors.
  # These methods will be called by self._step(). Each method may return None
  # indicating that no steering is required at this time for the behavior.

  def _steerForSeek(self, target=None):
    """Return the steering_direction (SteerVec) required to seek target.
    target should be a SteerVec."""

    if target == None: target = self._target

    desired_velocity = -(self._pos - target) * self.maxspeed
    desired_velocity.normalize()
    steering_direction = desired_velocity - self._velocity
    return steering_direction

  def _steerForFlee(self, target=None):
    """Return the steering_direction (SteerVec) required to flee target.
    target should be a SteerVec."""

    if target == None: target = self._target

    desired_velocity = (self._pos - target) * self.maxspeed
    desired_velocity.normalize()
    steering_direction = desired_velocity - self._velocity
    return steering_direction

  def _steerForPursue(self, target=None):
    """Return the steering_direction (SteerVec) required to pursue target.
    target should be another Vehicle."""

    # FIXME: Should the prediction interval be a parameter somewhere?

    if target == None: target = self._target

    prediction = target.predictFuturePosition(10)
    return self._steerForSeek(prediction)

  def _steerForFollow(self, target=None):
    """Return the steering_direction (SteerVec) required to follow target.
    target should be another Vehicle. Follow is like the pursue behavior
    but the vehicle will follow behind its target instead of catching up
    to it. The arrive behavior is place of seek in pursuit"""

    if target == None: target = self._target
    direction = SteerVec(target._velocity.getX(),target._velocity.getY())
    direction.normalize()
    point = target._pos + direction*-5
    return self._steerForArrive(point)

  def _steerForEvade(self, target=None):
    """Return the steering_direction (SteerVec) required to evade target.
    target should be another Vehicle."""

    # FIXME: Should the prediction interval be a parameter somewhere?

    if target == None: target = self._target

    prediction = target.predictFuturePosition(10)
    return self._steerForFlee(prediction)

  def _steerForArrive(self, target=None):
    """Return the steering_direction (SteerVec) required to arrive at
    target. target should be a SteerVec."""

    if target == None: target = self._target
    else:
      # Make sure it's a SteerVec (if it's not we assume it's something
      # with a getX() and getY() and we translate it)
      if not isinstance(target,SteerVec):
        target = SteerVec(target.getX(),target.getY())

    # FIXME: slowing_distance should be a parameter somewhere.
    slowing_distance = 20

    target_offset = target - self._pos
    distance = target_offset.length()
    if distance == 0: return SteerVec(0,0)
    ramped_speed = self.maxspeed * (distance / slowing_distance)
    clipped_speed = min(ramped_speed,self.maxspeed)
    desired_velocity = target_offset * (clipped_speed / distance)
    steering_direction = desired_velocity - self._velocity
    return steering_direction

  def _steerForWander(self):
    """Return a steering_direction for wandering."""
    speed = 0.2
    self.wanderSide = self._scalarRandomWalk(self.wanderSide,speed,-1,1)
    self.wanderUp   = self._scalarRandomWalk(self.wanderUp,speed,-1,1)
    up = self._velocity
    up.normalize() # The vehicle's forward direction
    side = up.rotate(90)
    return ((side * self.wanderSide) + (up * self.wanderUp)).truncate(0.01)

  def _scalarRandomWalk(self,initial,walkspeed,minimum,maximum):
    """Helper function for Vehicle._steerForWander() below."""

    next = initial + ((random.random()*2)-1)*walkspeed
    if next < minimum: return minimum
    if next > maximum: return maximum
    return next

  def _steerForAvoidObstacles(self):
    """
    Return a steering force to avoid the nearest obstacle that is
    considered a collision threat, or None to indicate that no obstacle
    avoidance steering is required.

    """
    # Check for collisions with obstacles.
    collision = False
    obstacleHandler.sortEntries()
    for i in range(obstacleHandler.getNumEntries()):
      entry = obstacleHandler.getEntry(i)
      if entry.getInto() == self.tube:
        collision = True
        pos = entry.getSurfacePoint(render)
        nrml = entry.getSurfaceNormal(render)

    if collision is False: return None

    # Compute steering to avoid the collision.
    nrml = SteerVec(nrml.getX(),nrml.getY())
    forward = self._velocity
    forward.normalize()
    steeringdirection = nrml.perpendicularComponent(forward)
    steeringdirection.normalize()
    brakingdirection = -self._velocity
    brakingdirection.normalize()
    md = self._velocity.length() + self.radius
    mf = self.maxforce
    p = SteerVec(pos.getX(),pos.getY())
    d = (self._pos - p).length()
    if d < 0: d = 0
    steeringforce = mf - ((d/md)*mf)
    brakingforce = mf - ((sqrt(d)/md)*mf)
    return (brakingdirection*brakingforce)+(steeringdirection*steeringforce)

  def _steerForContainment(self):
    """
    If this vehicle is outside either the global container or its local
    self.container return a steering force to turn the vehicle back towards
    the container. Else return None to indicate no containment steering is
    required.

    self.container is given priority over the global container.

    """
    global container
    for container in (self.container,container):
      if container is None: continue
      if not container.isInside(self._pos):
        # We are outside of the container, steer back toward it
        seek = self._steerForSeek(container.pos)
        lateral = seek.perpendicularComponent(self.getForward())
        return lateral
    return None

  def _steerForFollowPath(self):

    if (self._pos-self._path[self._waypoint]).length()<3:
      self._waypoint += 1
      if self._waypoint >= len(self._path):
        if self._loop: self._waypoint = 0
        else: self._waypoint = len(self._path)-1
    if not self._loop and self._waypoint == len(self._path)-1:
      return self._steerForArrive(self._path[self._waypoint])
    else:
      return self._steerForSeek(self._path[self._waypoint])

  def _steerForAvoidVehicles(self):
    """
    Return a steering force to avoid the site of the soonest potential
    collision with another vehicle, or None if there is no impending
    collision.

    Unaligned collision avoidance behavior: avoid colliding with other
    nearby vehicles moving in unconstrained directions.  Determine which
    (if any) other other vehicle we would collide with first, then steer to
    avoid the site of that potential collision.

    """
    # First priority is to steer hard to avoid very close vehicles
    steering = self._steerForAvoidCloseNeighbors(0)
    if steering is not False:
      return steering

    # Otherwise look for collisions further away.
    steer = 0
    threat = None

    # Time (in seconds) until the most immediate collision threat found
    # so far.  Initial value is a threshold: don't look more than this
    # many frames into the future.
    minTime = 60

    # Determine which (if any) neighbor vehicle poses the most immediate
    # threat of collision.
    for vehicle in vehicles:
      if vehicle is self: continue
      # Avoid when future positions are this close (or less)
      collisionDangerThreshold = self.radius * 2
      # Predicted time until nearest approach
      time = self._predictNearestApproachTime(vehicle)
      # If `time` is in the future, sooner than any other threatened
      # collision...
      if time >= 0 and time < minTime:
        # If the two will be close enough to collide make a note of it
        thrtPosAtNrstApprch,distance = self._computeNearestApproachPositions(time,vehicle)
        if distance < collisionDangerThreshold:
          minTime = time
          threat = vehicle
    # If a potential collision was found, compute steering to avoid
    if threat is None:
      return None
    else:
      parallelness = self.getForward().dot(threat.getForward())
      angle = 0.707
      if parallelness < -angle:
        # Anti-parallel "head on" paths: steer away from future threat
        # position
        offset = thrtPosAtNrstApprch - self._pos
        sideDot = offset.dot(self.getRight())
        if sideDot > 0:
          steer = -1
        else:
          steer = 1
      elif parallelness > angle:
        # Parallel paths: steer away from threat
        offset = threat._pos - self._pos
        sideDot = offset.dot(self.getRight())
        if sideDot > 0:
          steer = -1
        else:
          steer = 1
      elif threat.getspeed() <= self.getspeed():
        # Perpendicular paths: steer behind threat (only the slower of
        # the two does this)
        sideDot = self.getRight().dot(threat._velocity)
        if sideDot > 0:
          steer = -1
        else:
          steer = 1

      return self.getRight() * steer

  def _predictNearestApproachTime(self,vehicle):
    """Return the time until nearest approach between this vehicle and
    another vehicle."""

    # Imagine we are at the origin with no velocity, compute the relative
    # velocity of the other vehicle
    myVelocity = self._velocity
    hisVelocity = vehicle._velocity
    relVelocity = hisVelocity - myVelocity
    relSpeed = relVelocity.length()

    # For parallel paths, the vehicles will always be at the same distance,
    # so return 0 (aka "now") since "there is no time like the present"
    if relSpeed == 0: return 0

    # Now consider the path of the other vehicle in this relative
    # space, a line defined by the relative position and velocity.
    # The distance from the origin (our vehicle) to that line is
    # the nearest approach.

    # Take the unit tangent along the other vehicle's path
    relTangent = relVelocity / relSpeed

    # Find distance from its path to origin (compute offset from
    # other to us, find length of projection onto path)
    relPosition = self._pos - vehicle._pos
    projection = relTangent.dot(relPosition)

    return projection / relSpeed

  def _computeNearestApproachPositions(self, time, vehicle):
    """Return a tuple containing the position of 'vehicle' at its nearest
    approach to this vehicle and the distance between the two at that
    point, given the time until the nearest approach of the two."""

    myTravel = (self.getForward() * self.getspeed()) * time
    hisTravel = vehicle.getForward() * vehicle.getspeed() * time
    myFinal = self._pos + myTravel
    hisFinal = vehicle._pos + hisTravel
    distance = (myFinal - hisFinal).length()
    return (hisFinal,distance)

  def _steerForAvoidCloseNeighbors(self, criticalDistance):
    """Avoidance of "close neighbors" -- used only by
     _steerForAvoidNeighbors. Does a hard steer away from any other vehicle
     who comes withing a critical distance."""

    for vehicle in vehicles:
      if vehicle is self: continue # No need to avoid yourself
      sumOfRadii = self.radius + vehicle.radius
      minCenterToCenter = criticalDistance + sumOfRadii
      offset = vehicle._pos - self._pos
      currentDistance = offset.length()
      if currentDistance < minCenterToCenter:
        return (-offset).perpendicularComponent(self.getForward())
    return False

  def _step(self,task):
    """
    Update the vehicle's position using a simple point-mass physics
    model based on self._steeringforce, self.maxforce, self.mass, and
    self.maxspeed, and applying the steering behavior indicated by
    self._steeringbehavior.

    """
    dt = task.time - self._prevtime

    steeringbehavior = self._steeringbehavior
    if self.avoidObstacles:
      steeringbehavior = ['AvoidObstacles'] + steeringbehavior
    if self.avoidVehicles:
      steeringbehavior = ['AvoidVehicles'] + steeringbehavior
    if (self.container is not None) or (container is not None):
      steeringbehavior = ['Containment'] + steeringbehavior

    # Set self._steeringforce to be the result of the first method in
    # steeringbehavior that does not return None.
    self._steeringforce = SteerVec(0,0)
    self.label['text'] = 'no steering'
    for steeringmethod in steeringbehavior:
      method_name = '_steerFor' + steeringmethod
      method = getattr(self, method_name)
      steeringforce = method()
      if steeringforce is not None:
        self.label['text'] = steeringmethod
        self._steeringforce = steeringforce
        break

    global show
    if show:
      self.label.show()
      #self.tubenp.show()
      #self.spherenp.show()
    else:
      self.label.hide()
      #self.tubenp.hide()
      #self.spherenp.hide()

    # Compute new velocity according to point-mass physics.

    steering_force = self._steeringforce.truncate(self.maxforce)
    acceleration = steering_force/self.mass
    self._velocity = (self._velocity+acceleration).truncate(self.maxspeed)

    # Add a small random component to the velocity. The component is too
    # small to produce any visible result. It is added to avoid a bug in
    # the obstacle avoidance behavior: if a vehicle heads precisely
    # toward an obstacle then the surface normal at the point of
    # collision on the surface of the obstacle is parallel to the forward
    # direction of the vehicle. So the perpendicular component of the
    # surface normal to the vehicle's forward direction is zero, and hence
    # the steering force is zero, and the vehicle plows straight into the
    # obstacle. With a random element to the vehicle's velocity, this
    # precise situation will not exist for consecutive simulation frames.
    x = (random.random()-0.5)*0.01
    y = (random.random()-0.5)*0.01
    self._velocity.setX(self._velocity.getX()+self._velocity.getX()*x)
    self._velocity.setY(self._velocity.getY()+self._velocity.getY()*y)

    # Update vehicle's position.
    self._pos += self._velocity*(dt*50)
    self._enforceNonPenetrationConstraint()

    # Fluidly update the NodePath's X and Y positions to match self._pos.
    # Leave the NodePath's Z-axis position untouched (that is taken care of
    # by CollisionHandlerFloor).
    self.prime.setFluidPos(self._pos.getX(),self._pos.getY(),
                 self.prime.getZ())

    # Update the Vehicles's CollisionTube, varying the length of the tube
    # according to the Vehicle's speed.
    ##zuck
    ###tube=self.tubenp.node().getSolid(0)
    tube=self.tubenp.node().modifySolid(0)
    tube.setPointB(Point3(
           tube.getPointA()+Point3(0,15.*self._velocity.length(),0)))

    self._prevtime = task.time
    return Task.cont

  def _enforceNonPenetrationConstraint(self):
    """Forcibly prevent this vehicle from penetrating obstacles.

    Based on advice from Craig Reynolds:
    The obstacle and vehicle are both assumed to be spheres. Measure the
    distance between their centers. If this is less than the sum of their
    radii the two spheres intersect. If so, simply "slide" (that is, SET
    the position of) the vehicle along the line connecting their centers
    the distance by which they overlap. See code here:
    http://sourceforge.net/forum/message.php?msg_id=3928822

    """

    # Handle collisions with obstacles.
    obstacleHandler.sortEntries()
    for i in range(obstacleHandler.getNumEntries()):
      entry = obstacleHandler.getEntry(i)
      if entry.getInto() == self.sphere:
        other = entry.getFrom()
        pos = other.getCenter()
        pos = SteerVec(pos.getX(),pos.getY())
        s = self.radius + other.getRadius()
        overlap = self._pos-pos
        d = overlap.length()
        if d>0: s = (s-d)/d
        overlap *= s
        self._pos += overlap
        return

    # Handle collisions with other vehicles.
    collisionHandler.sortEntries()
    for i in range(collisionHandler.getNumEntries()):
      entry = collisionHandler.getEntry(i)

      # FIXME: The collisionmasks in this module aren't quite right.
      # Want Vehicle.sphere to collide with other Vehicle.sphere's and
      # with obstacles.py/SphereObstacle's, and want Vehicle.tube to
      # collide with obstacles.py/SphereObstacle's, but we don't want
      # Vehicle.sphere to collide with the Vehicle.tube of another
      # vehicle! (or vice-versa)
      # For now just check for the unwanted tube collisions and ignore
      # them.
      if isinstance(entry.getInto(),CollisionTube): continue

      if entry.getFrom() == self.sphere:
        pos = entry.getIntoNodePath().getPos(render)
        pos = SteerVec(pos.getX(),pos.getY())
        s = self.radius + entry.getInto().getRadius()
        overlap = self._pos-pos
        d = overlap.length()
        if d>0: s = (s-d)/d
        overlap *= s
        self._pos += overlap
        if self.callout['text'] == '':
          self.callout['text'] = random.choice(["Ouch!","Sorry!","Hey!"])
          self.callout.show()
        return
    self.callout['text']=''
    self.callout.hide()
