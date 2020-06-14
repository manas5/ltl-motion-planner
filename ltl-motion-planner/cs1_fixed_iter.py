#!/usr/bin/env python
'''
.. module:: example
   :synopsis: Defines the case study presented in the IJRR journal paper.

'''

import os, sys
import logging
import itertools as it
import operator as op
import numpy as np
import math

from spaces.base import Workspace, line_translate
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robot import Robot

from sparse_rrg import RRGPlanner
from product import IncrementalProduct
from lomap import Ts,compute_potentials, Timer
from planar import addStyle, Simulate2D, to_rgba


def setup(outputdir='.', logfilename='example.log'):
    '''Setup logging, and set output and debug options.'''
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logfile = os.path.join(outputdir, logfilename)
    verbose = True
    logging.basicConfig(filename=logfile, filemode='w', level=loglevel,
                        format=fs, datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)


def define_problem(outputdir='.'):
    '''Define case study setup (robot parameters, workspace, etc.).'''

    # define robot diameter (used to compute the expanded workspace)
    robotDiameter = 0.02

    # define boundary
    boundary = BoxBoundary2D([[0, 1], [0, 1]])
    # define boundary style
    boundary.style = {'color' : 'black'}
    # create expanded region
    eboundary = BoxBoundary2D(boundary.ranges +
                                np.array([[1, -1], [1, -1]]) * robotDiameter/2)
    eboundary.style = {'color' : 'black'}

    # create robot's workspace and expanded workspace
    wspace = Workspace(boundary=boundary)
    ewspace = Workspace(boundary=eboundary)

    # create robot object
    robot = Robot('vector', init=Point2D((0.3, 0.3)), wspace=ewspace,
                               stepsize=0.168)
    robot.diameter = robotDiameter

    logging.info('"Workspace": (%s, %s)', wspace, boundary.style)
    logging.info('"Expanded workspace": (%s, %s)', ewspace, eboundary.style)
    logging.info('"Robot name": "%s"', robot.name)
    logging.info('"Robot initial configuration": %s', robot.initConf)
    logging.info('"Robot step size": %f', robot.controlspace)
    logging.info('"Robot diameter": %f', robot.diameter)
    # create simulation object
    sim = Simulate2D(wspace, robot, ewspace)
    sim.config['output-dir'] = outputdir

    # regions of interest
    R1 = (BoxRegion2D([[0, 0.2], [0, 0.2]], ['r1']), 'green')
    R2 = (BoxRegion2D([[0.8,1], [0, 0.2]], ['r2']), 'green')
    R3 = (BoxRegion2D([[0.8, 1], [0.8,1]], ['r3']), 'green')
    R4 = (BoxRegion2D([[0, 0.2], [0.8,1]], ['r4']), 'green')
    
    # global obstacles
    O1 = (BoxRegion2D([[0.4,0.6], [0, 0.1]], ['o1']), 'red')
    O3 = (BoxRegion2D([[0, 0.1], [0.4,0.6]], ['o1']), 'gray')
    O2 = (BoxRegion2D([[0.9, 1], [0.4,0.6]], ['o2']), 'gray')
    O4 = (BoxRegion2D([[0.4,0.6], [0.4,0.6]], ['o4']), 'red')

    # add all regions
    regions = [R1, R2, R3, R4, O1, O2, O3, O4]

    # add regions to workspace
    for k, (r, c) in enumerate(regions):
        # add styles to region
        addStyle(r, style={'facecolor': c})
        # add region to workspace
        sim.workspace.addRegion(r)
        # create expanded region
        er = expandRegion(r, robot.diameter/2)
        # add style to the expanded region
        addStyle(er, style={'facecolor': c})
        # add expanded region to the expanded workspace
        sim.expandedWorkspace.addRegion(er)

        logging.info('("Global region", %d): (%s, %s)', k, r, r.style)


    # display workspace
    sim.display()

    # display expanded workspace
    sim.display(expanded=True)

    ltlSpec = ('[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4) && '
                                                   '!(o1 || o2 || o3 || o4))')
    logging.info('"Global specification": "%s"', ltlSpec)


    return robot, sim, ltlSpec

def generate_ts(ltlSpec, sim, robot, eta=[0.5, 1.0],
                       ts_file='ts.yaml', outputdir='.'):
    '''Generate global transition system and off-line control policy.'''

    # initialize incremental product automaton
    checker = IncrementalProduct(ltlSpec) #, specFile='ijrr_globalSpec.txt')
    logging.info('"Buchi size": (%d, %d)', checker.buchi.g.number_of_nodes(),
                                           checker.buchi.g.number_of_edges())

    
    

    # initialize global off-line RRG planner
    sim.offline = RRGPlanner(robot, checker, iterations=200, eta=eta)
    suffix_cost = None

    for session in range(1,6):
        logging.info('"Global Planning session %d started"', session)
        found = False
        with Timer(op_name='Session',template='"%s runtime": %f ms'):
            found = sim.offline.solve()
            logging.info('"Found solution in session %d": %s', session, found) 
            logging.info('"Iterations": %d', sim.offline.iteration)
            logging.info('"Size of TS": %s', sim.offline.ts.size())
            logging.info('"Size of PA": %s', sim.offline.checker.size())

    # save transition system and control policy
    if ts_file is not None:
        sim.offline.ts.save(os.path.join(outputdir, ts_file))
    prefix, suffix, suffix_cost  = sim.offline.checker.globalPolicy(sim.offline.ts)
    logging.info('"global policy": (%s, %s)', prefix, suffix)
    # sim.display(solution=([v[0] for v in prefix],[v[0] for v in suffix]), save='global_sol')
    logging.info('"Suffix cost"%s',suffix_cost)
    logging.info('"End global planning": True')

    return found

if __name__ == '__main__':
    outputdir = os.path.abspath('../data/cs1/fixed_iter')
    setup(outputdir)
    robot, sim, ltlSpec = define_problem(outputdir)
    np.random.seed(1001)
    with Timer(op_name='Global Planning',template='%s runtime: %f ms'):
        generate_ts(ltlSpec, sim, robot, eta=[0.042,0.169], ts_file='ts.yaml')
