'''
.. module:: sparse_rrg
   :synopsis: The module implements the RRG based path planner with LTL
    constraints. The algorithm represents the (off-line) global component of the
    proposed framework.

'''

import logging
from itertools import tee

from lomap import Ts, Timer


class RRGPlanner(object):
    '''
    Class implementing the RRG based path planner with LTL constraints.
    The algorithm represents the (off-line) global component of the proposed
    framework.
    '''

    def __init__(self, robot, checker, iterations=0, eta=[0.5, 1.0]):
        '''Constructor'''
        self.name = 'Sparse RRG-based LTL path planner'
        self.maxIterations = iterations
        self.iteration = 0

        self.robot = robot
        symbols = self.robot.getSymbols(robot.initConf)

        # initialize transition system
        self.ts = Ts(name='Transition system', directed=True, multi=False)
        self.ts.g.add_node(robot.initConf, prop=symbols)
        self.ts.init[robot.initConf] = 1.0

        # initialize checker
        self.checker = checker
        if checker:
            self.checker.addInitialState(robot.initConf, symbols)

        self.eta = eta


    def solve(self, finite = False):
        '''Try to solve the problem.'''
        if finite and self.checker.foundPolicy():
            return True
        for self.iteration in range(1, self.maxIterations+1):
            logging.info('"iteration": %d', self.iteration)
            self.iterate(self.iteration)
            if finite and self.checker.foundPolicy():
            	return True
        return self.checker.foundPolicy()

    def iterate(self, iteration):
        '''Execute one step of the off-line global planner.'''
        ### First phase - Forward
        # initialize update sets
        newState, Delta, E = None, set(), set()
        # sample new configuration
        randomConf = self.robot.sample()
        nearestState = self.nearest(randomConf)
        # steer towards the random configuration
        newConf = self.robot.steer(nearestState, randomConf)
        # set propositions
        newProp = self.robot.getSymbols(newConf)



        logging.info('"random configuration": %s', randomConf)
        logging.info('"nearest state": %s', nearestState)
        logging.info('"new configuration": %s', newConf)

        for state in self.far(newConf):
            # check if the new state satisfies the global specification
            if self.robot.isSimpleSegment(state, newConf):
            	weight = self.robot.weight(state, newConf)
                Ep = self.checker.check(self.ts, state, newConf,
                                            newProp, weight, forward=True)
                if Ep:
                    newState = newConf
                    Delta.add((state, newConf,weight))
                    E.update(Ep)

        if newState:
            self.ts.g.add_node(newState, prop=newProp)
            self.ts.g.add_weighted_edges_from(Delta)
            self.checker.update(E)

        logging.info('"forward state added": %s', newState)
        logging.info('"forward edges added": %s', Delta)

        ### Second phase - Backward
        Delta = set()
        E = set()
        if newState: # for newly added state
            for state in self.near(newState):
                st = self.robot.steer(newState, state, atol=1e-8)
                # if the robot can steer from a new state to another state
                if (state == st) and self.robot.isSimpleSegment(newState, state):
                    # check if the new state satisfies the global
                    # specification
                    weight = self.robot.weight(newState, state)
                    Ep = self.checker.check(self.ts, newState, state,
                               self.ts.g.node[state]['prop'], weight, forward=False)
                    if Ep:
                        Delta.add((newState, state, weight))
                        E.update(Ep)

        self.ts.g.add_weighted_edges_from(Delta)
        self.checker.update(E)
        logging.info('"backward edges added": %s', Delta)

        return self.checker.foundPolicy()


    def nearest(self, p):
        '''Returns the nearest configuration in the transition system.'''
        dist = self.robot.cspace.dist
        return min(self.ts.g.nodes_iter(), key=lambda x: dist(p, x))

    def far(self, p):
        '''Return all states in the transition system that fall at distance d,
        d < self.eta[1], away from the given configuration p. If there is a
        state which is closer to the given configuration p than self.eta[0]
        then the function returns an empty list.
        '''
        logging.info('"far": (%s, %f, %f)', p, self.eta[0], self.eta[1])
        metric = self.robot.cspace.dist
        ret, test = tee(filter(lambda v: metric(v, p) < self.eta[1],
                                self.ts.g.nodes_iter()))
        if any(map(lambda v: metric(v, p) <= self.eta[0], test)):
            return iter([])
        return ret

    def near(self, p):
        '''Return all states in the transition system that fall at distance d,
        0 < d < self.eta[1], away from the given configuration p.
        '''
        logging.info('("near", %s): %f', p, self.eta[1])
        metric = self.robot.cspace.dist
        return filter(lambda v: 0 < metric(v, p) < self.eta[1],
                       self.ts.g.nodes_iter())

    def update_iter(self, new_iterations):
    	self.maxIterations = new_iterations
    	return

    def add_to_ts(self, E):
    	for _,state,_ in E:
    		self.ts.g.add_node(state, prop=self.robot.getSymbols(state))
    	self.ts.g.add_weighted_edges_from(E)	

    def update(self,eta):
    	self.eta = eta
    	return