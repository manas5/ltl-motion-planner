'''
.. module:: Robot
   synopsis: Module implements a fully actuated robot (an integrator).

'''


class Robot(object):
    '''
    Class representing a robot.
    '''
    def __init__(self, name, init=None, cspace=None, wspace=None, stepsize=0.1):
        self.name = name

        self.initConf = init # initial position
        self.currentConf = self.initConf # current position

        self.cspace = wspace
        self.wspace = wspace
        self.controlspace = stepsize

    def sample(self, local=False):
        ''' Generate a sample in the configuration space (global) or local
        within the sensing area.
        '''
        return self.cspace.getSample()

    def move(self, conf):
        '''Moves the robot to the given configuration.'''
        self.currentConf = conf
    

    def getSymbols(self, position):
        '''Returns the symbols satisfied at the given position.
        '''
        return self.wspace.getSymbols(position)

    def steer(self, start, target, atol=0):
        '''Returns a position that the robot can move to from the start position
        such that it steers closer to the given target position using the
        robot's dynamics.

        Note: It simulates the movement.
        '''
        s = start.coords
        t = target.coords
        dist = self.cspace.metric(s, t)
        if dist <= self.controlspace + atol:
            return target
        return self.initConf.__class__(s + (t-s) * self.controlspace/dist)

    def weight(self, start, target, atol=0, weight_factor=1.0):
        '''Returns the weight of edge between the start position and target position.
        The weight is calculated based on Euclidean distance between the start 
        position and target position.
        '''
        s = start.coords
        t = target.coords
        return weight_factor*self.cspace.metric(s, t)

    def isSimpleSegment(self, u, v):
        '''Returns True if the curve [x, y] = H([u, v]) in the workspace space
        does not intersect other regions than the ones x and y belong to.
        '''
        nrRegUV = len(self.cspace.intersectingRegions(u, v))
        regU = self.cspace.intersectingRegions(u)
        regV = self.cspace.intersectingRegions(v)

        # if both endpoints of the segment are in the free space
        if (not regU) and (not regV):
            return nrRegUV == 0
        # if one endpoint is in the free space
        if (not regU) or (not regV): # NOTE: assumes convex regions
            return nrRegUV == 1
        # if both endpoints are in the same region
        if regU[0] == regV[0]:
            # NOTE: experimental for non-convex regions
            # and regU.contains(u, v):
            return nrRegUV == 1
        # if the endpoints belong to two different regions
        return nrRegUV == 2

    def update_param(self, name=None, workspace=None, init=None, stepsize=None):
        if name:
            self.name = name
        if workspace:
            self.wspace = workspace
            self.cspace = workspace
        if init:
            self.initConf = init
        if stepsize:
            self.controlspace = stepsize

    def __str__(self):
        return 'Fully actuated robot'
