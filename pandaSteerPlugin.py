"""
pandaSteerPlugin.py

Default PandaSteer plugin, from which all other plugins should inherit to add
interesting behaviour.

This module defines a class PandaSteerPlugin. To create a plugin module that
will be loaded by PandaSteer you should define a class Plugin that inherits
from PandaSteerPlugin. Place your plugin module in the plugins folder, and
import PandaSteerPlugin as if it were in the same folder, like this:

from pandaSteerPlugin import PandaSteerPlugin

Your class **must** be called Plugin, this is not just a suggestion, but you
can call your module whatever you want as long as it's in the plugins folder.
If your Plugin class overrides __init__ it should make a call to
PandaSteerPlugin.__init__(self). This is not necessary for any other methods.

You can look at some of the implemented plugins for a guide.

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

import random
from steerVec import SteerVec

class PandaSteerPlugin:
    """PandaSteer default plugin from which all other plugins should inherit."""
    
    def __init__(self):
        """Initialise the plugin. In this method Your plugin should initialise
        any of the values you see here, if you want them to be different from
        the defaults. **Do not** assign positions, velocities, steering
        behaviours etc. to vehicles in this method, that should be done in
        self.restart.
        """
    
        self.numVehicles = 0 # The number of vehicles (characters) in the scene
        self.vehicles = [] # List that holds the vehicles
        self.wrap = False # Wrap vehicles around if they leave the terrain?
        self.obstaclesOn = False # Place obstacles in the scene?
        self.obstacles = [] # List that holds the obstacles
        self.container = None # Container that all characters will stay within
        self.text = """PandaSteer default plugin.""" # On-screen text

    def click(self,pos):
        """Respond to mouse-click. pos is the (X,Y,Z) location on the terrain
        that the user clicked."""

        pass        
    
    def restart(self):
        """Start or restart the plugin. This is where you assign initial
        positions, velocities, steering behaviours etc. to vehicles."""
        
        pass
