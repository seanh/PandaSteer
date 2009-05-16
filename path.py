class Path:
    """A list of points representing a path."""

    def __init__(self,points):
        """Intialise the path object."""
        
        self.points = points
        
    def getNearestWaypoint(self,p):
        """Return the index of the nearest waypoint on this path to arbitrary
           point p."""
        
        nearest = 0
        d = (p-self.points[nearest]).length()
        for index,point in enumerate(self.points[1:]):
            if (p-point).length() < d:
                nearest = index+1
                d = (p-point).length()
        return nearest
        
    # FIXME: This class should probably extend the standard list class
    def __getitem__(self,key):
        return self.points[key]
    def __setitem__(self,key,value):
        self.points[key]=value
    def __len__(self):
        return len(self.points)
