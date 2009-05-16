"""
character.py -- An animated player or non-player character that moves over
                uneven terrain with various steering behaviours.

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

# TODO: Decide which functions and variables are public and which private in
# Character, and set the variable names accordingly.

# Panda3D imports
import direct.directbase.DirectStart
from pandac import PandaModules as P
from direct.fsm import FSM
from direct.actor.Actor import Actor
from direct.task.Task import Task
# Custom imports
from vehicle import Vehicle,floorMASK,obstacleMASK,vehicleMASK,offMASK,toggleShow,setContainer,unsetContainer

annotation = False # Toggle annotation of Characters on/off
def toggleAnnotation():
    global annotation
    if annotation:
        annotation = False
    else:
        annotation = True
    toggleShow()

class Character(FSM.FSM,Vehicle):
    """
    An animated character with various steering behaviors. By enabling and
    disabling different steering behaviors Character can be used as
    a keyboard + mouse controlled player avatar, or can be controlled (for
    example) by a Finite State Machine and used to implement a non-player
    character.
    """

    # FIXME: Think it might be clearer and safer if Character has a FSM instead
    # of inheriting from FSM (classes that inherit from Character will likely
    # want to inherit from FSM too).

    def __init__(self,name='Ralph',model='models/ralph',run='models/ralph-run',
                 walk='models/ralph-walk',pos=None,avoidObstacles=True,
                 avoidVehicles=True,
                 hprs=(180,0,0,1,1,1), # Ralph's Y is backward
                 ):
        """Initialise the character.

        By default tries to load Panda3D's Ralph model: models/ralph,
        models/ralph-run and models/ralph-walk."""

        FSM.FSM.__init__(self,'Character')
        Vehicle.__init__(self,pos=pos,avoidObstacles=avoidObstacles,
                         avoidVehicles=avoidVehicles,radius=2.5)

        self.name = name
        self.lastPose = 0 # Used when transitioning between animations
        self.actor = Actor(model,{"run":run,"walk":walk})        
        self.actor.setHprScale(*hprs)
        self.actor.reparentTo(self.prime)

        # Add a task for this Character to the global task manager.
        self.characterStepTask=taskMgr.add(self.characterStep,"Character step task")

    def characterStep(self,task):
        """Update the character. Called every frame by Panda's global task
        manager."""

        # Update the orientation of the Character. Want to face in the
        # direction of the Vehicle's velocity in the X and Y dimensions, but
        # always face straight forward (not up or down) in the Z dimension.
        pr = self.prime.getP(),self.prime.getR()
        self.prime.lookAt((self._pos+self._velocity).getX(),
                          (self._pos+self._velocity).getY(),0)
        self.prime.setHpr(self.prime.getH(),*pr)

        # Animate the Character's Actor. The Character automatically changes
        # between stand, run and walk animations and varies the playrate of the
        # animations depending on the speed of the Character's Vehicle.
        speed = self.getspeed()
        if speed < .05:
            if self.getCurrentOrNextState() != 'Stand':
                self.request('Stand')
        elif speed < .25:
            self.actor.setPlayRate(speed*4,"walk")
            if self.getCurrentOrNextState() != 'Walk':
                self.request('Walk')
        else:
            self.actor.setPlayRate(speed*2,"run")
            if self.getCurrentOrNextState() != 'Run':
                self.request('Run')

        return Task.cont

    def destroy(self):
        """Prepare this Character to be garbage-collected by Python:
        
        Remove all of the Character's nodes from the scene graph, remove its
        CollisionSolids from the global CollisionTraverser, clear its
        CollisionHandler, remove tasks, destroy Actor.
        
        After executing this method, any remaining references to the Character
        object can be destroyed by the user module, and the Character will be
        garbage-collected by Python.
        """
    
        taskMgr.remove(self.characterStepTask)
        self.cleanup()
        self.actor.delete()
        Vehicle.destroy(self)

    # Methods for handling animations.

    def storeLastPose(self):
        currAnim=self.actor.getCurrentAnim()
        numFrames=self.actor.getNumFrames(currAnim)
        animFrame=self.actor.getCurrentFrame(currAnim)
        self.lastPose=float(animFrame)/float(numFrames)
        self.actor.stop(currAnim)

    def loopFromPose(self,animName):
        self.actor.pose(animName,frame=self.lastPose*self.actor.getNumFrames(animName))
        self.actor.loop(animName,restart=0)

    # FSM State handlers. Called when transitioning to a new state.
    def enterRun(self):  self.loopFromPose("run")
    def exitRun(self):   self.storeLastPose()
    def enterWalk(self): self.loopFromPose("walk")
    def exitWalk(self):  self.storeLastPose()
    def enterStand(self):
        standPoseFrame=6    # frame 6 (the most acceptable stand pose)
        numFrames=self.actor.getNumFrames("walk")
        lastFrame=self.lastPose*numFrames
        # "mirror" the frame to bring it closer to the most acceptable stand pose
        if lastFrame>.5*(numFrames-1):
           lastFrame=numFrames-1-lastFrame
        frameDiff=standPoseFrame-lastFrame
        # if already at stand pose, don't do anything
        if frameDiff==0:
           return
        # forward animation playback
        if frameDiff>=0:
           fromFrame=lastFrame
           toFrame=standPoseFrame
        else:
        # backward animation playback
           fromFrame=standPoseFrame
           toFrame=lastFrame
        playDir=2*frameDiff/numFrames
        self.actor.setPlayRate(playDir,"walk")
        self.actor.play("walk", fromFrame=fromFrame, toFrame=toFrame)
