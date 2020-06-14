'''
.. module:: planar
   :synopsis: Module defining the classes for displaying planar environments.
.. 
'''

import os
import itertools as it
import logging

import numpy as np
from numpy import mean

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ColorConverter
from matplotlib.collections import LineCollection

from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, BallBoundary2D, BoxBoundary2D, PolygonBoundary2D, Point2D, expandRegion


__all__ = []

colconv = ColorConverter()
to_rgba = colconv.to_rgba


def drawPoint2D(viewport, point, color, style=None):
    '''Draws a point in the planar environment.'''
    if style:
        viewport.plot(point.x, point.y, color=color, **style)
    else:
        viewport.plot(point.x, point.y, color=color)

def drawBoxRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a box region in a planar environment.'''
    x = [region.ranges[0, 0], region.ranges[0, 0],
         region.ranges[0, 1], region.ranges[0, 1]]
    y = [region.ranges[1, 0], region.ranges[1, 1],
         region.ranges[1, 1], region.ranges[1, 0]]
    viewport.fill(x, y, **style)
    if text:
        if textStyle:
            viewport.text(mean(x), mean(y), text, **textStyle)
        else:
            viewport.text(mean(x), mean(y), text)

def drawBallRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a ball region in a planar environment.'''
    x, y = region.center
    c = plt.Circle(region.center, region.radius, **style)
    viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawPolygonRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a polygonal region in a planar environment.'''
    x, y = zip(*region.polygon.exterior.coords)
    viewport.fill(x, y, **style)
    if text:
        x, y = region.polygon.centroid.coords[0]
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a region in a planar environment.'''
    if isinstance(region, BoxRegion2D):
        drawBoxRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, BallRegion2D):
        drawBallRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, PolygonRegion2D):
        drawPolygonRegion2D(viewport, region, style, text, textStyle)
    else:
        raise TypeError

def drawBoundary2D(viewport, boundary, style):
    '''Draws a boundary in a planar environment.'''
    if isinstance(boundary, BoxBoundary2D):
        x, y = zip(*((0, 0), (0, 1), (1, 1), (1, 0), (0, 0)))
        x, y = boundary.xrange()[list(x)], boundary.yrange()[list(y)]
        viewport.plot(x, y, **style)
    elif isinstance(boundary, BallBoundary2D):
        c = plt.Circle(boundary.center, boundary.radius, **style)
        viewport.add_artist(c)
    elif isinstance(boundary, PolygonBoundary2D):
        x, y = zip(*boundary.polygon.exterior.coords)
        viewport.plot(x, y, **style)

def addStyle(region, style=None, withText=True, text=None, textStyle=None):
    '''Adds style information to the region for rendering.'''
    if style:
        region.style = style
        region.style['facecolor'] = style.get('facecolor', 'white')
        region.style['edgecolor'] = style.get('edgecolor', 'black')
    else:
        region.style = {'facecolor': 'white', 'edgecolor':'black'}

    if withText:
        if text:
            region.text = text
        else:
            region.text = ' '.join(region.symbols)
        if textStyle:
            region.textStyle = textStyle
            region.textStyle['horizontalalignment'] = \
                                  textStyle.get('horizontalalignment', 'center')
            region.textStyle['verticalalignment'] = \
                                    textStyle.get('verticalalignment', 'center')
            region.textStyle['fontsize'] = textStyle.get('fontsize', 12)
        else:
            region.textStyle = {'horizontalalignment' : 'center',
                               'verticalalignment' : 'center',
                               'fontsize' : 12}
    else:
        region.text = None
        region.textStyle = None

def drawRobot2D(viewport, robot, size, text='', textStyle=None,
                sensing_area=True, zorder=2):
    '''Draws the robot with its sensing area in the viewport.'''
    x, y = robot.currentConf.coords

    style = {'facecolor': 'blue', 'edgecolor':'black', 'zorder': zorder}
    if size == 0:
        plt.plot(x, y, 'o', color=style['facecolor'], zorder=zorder)
    else:
        c = plt.Circle(robot.currentConf.coords, size, **style)
        viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawGraph(viewport, g, node_color='blue', edge_color='black', zorder=2):
    '''Plots the given graph in the viewport.'''
    x, y = zip(*[(u.x, u.y) for u in g.nodes_iter()])
    viewport.scatter(x, y, c=node_color, zorder=zorder, s=50)

    lines = [[(u.x, u.y), (v.x, v.y)] for u, v in g.edges_iter()]
    artist = LineCollection(lines, colors=edge_color, zorder=zorder)
    viewport.add_collection(artist)

def drawPolicy(viewport, g, solution, color='black', alpha_min=1.0, zorder=2):
    '''Draws the solution path with a fading effect.'''

    for u, v in it.izip(solution[0], solution[0][1:]):
        dx, dy = v.x - u.x, v.y - u.y
        viewport.scatter(u.x, u.y, c=color, zorder=zorder, s=50)
        # viewport.annotate(g.node[u]['num'],(u.x,u.y), textcoords="offset points", xytext=(0,10), zorder=zorder+1)
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color, alpha=alpha_min/2,
                  length_includes_head=True, head_width=0.01, zorder=zorder)

    for u, v in it.izip(solution[1], solution[1][1:]):
        dx, dy = v.x - u.x, v.y - u.y
        viewport.scatter(u.x, u.y, c=color, zorder=zorder)
        # viewport.annotate(g.node[u]['num'],(u.x,u.y), textcoords="offset points", xytext=(0,10), ha='center', zorder=zorder+1)
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color, alpha=alpha_min,
                  length_includes_head=True, head_width=0.01, zorder=zorder)


class Simulate2D(object):
    '''Management class for planar systems and environments. It implements
    visualization of the workspace, saves images or videos, and support playback
    of computed trajectories.

    It also handles the simulation of local requests.
    '''

    def __init__(self, workspace, robot, expandedWorkspace=None, config=None):
        self.workspace = workspace
        self.expandedWorkspace = expandedWorkspace
        self.robot = robot

        if config:
            self.config = config
        else:
            self.config = dict()
        self.defaultConfiguration()

        self.offline = None

        self.solution = None

    def defaultConfiguration(self, reset=None):
        '''Sets default values for various parameters.'''
        if reset: # set all parameters in reset to their default values
            for key in reset:
                if key in self.config:
                    del self.config[key]

        self.config['x-padding'] = self.config.get('x-padding',
                                                   self.robot.diameter/2)
        self.config['y-padding'] = self.config.get('y-padding',
                                                   self.robot.diameter/2)
        self.config['grid-on'] = self.config.get('grid-on', True)
        self.config['sim-step'] = self.config.get('sim-step', 0.1)

        self.config['detected-request-transparency'] = \
                        self.config.get('local-obstacle-transparency', 0.2)
        self.config['request-transparency'] = \
                        self.config.get('request-transparency', 0.5)

        self.config['global-ts-color'] = {'node_color': 'gray',
                                          'edge_color': 'gray'}
        self.config['global-policy-color'] = 'black'

        self.config['background-transparency'] = 1.0
        self.config['image-file-template'] = \
          self.config.get('image-file-template', 'frames/frame_{frame:04d}.png')
        self.config['output-dir'] = self.config.get('output-dir', '.')

        self.config['z-order'] = {
            'background'          : 0,
            'boundary'            : 1,
            'global region'       : 2,
            'global ts'           : 3,
            'global region text'  : 4,
            'global plan'         : 10,
            'robot'               : 15,
            }

    def loadConfiguration(self, filename):
        pass

    def saveConfiguration(self, filename):
        pass

    def loadTimeline(self, filename):
        pass

    def reset(self):
        pass

    def display(self, expanded=False, solution=None, localinfo=None,
                save=None, show_robot=True, figsize=None):
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, aspect='equal')

        if expanded == 'both':
            self.render(ax, expanded=True, solution=None, localinfo=localinfo,
                        show_robot=False)
            self.render(ax, expanded=False, solution=solution,
                        localinfo=localinfo, show_robot=show_robot)
        else:
            self.render(ax, expanded, solution, localinfo=localinfo,
                        show_robot=show_robot)

        if save is not None:
            plt.subplots_adjust(left=0.05, bottom=0.05, right=0.98, top=0.98,
                                wspace=0, hspace=0)
            plt.savefig(os.path.join(self.config['output-dir'], save),
                        dpi=fig.dpi)
        plt.show(block=False)
        plt.pause(3)
        plt.close()

    def render(self, viewport, expanded=False, solution=None, withtext=True,
               localinfo=None, sensing_area=False, show_robot=True):
        zorder = self.config['z-order']

        if expanded:
            wp = self.expandedWorkspace
        else:
            wp = self.workspace
        wp.boundary.style['zorder'] = zorder['boundary']

        limits = wp.boundary.boundingBox()
        plt.xlim(limits[0] + np.array([-1, 1]) * self.config['x-padding'])
        plt.ylim(limits[1] + np.array([-1, 1]) * self.config['y-padding'])

        if self.config.get('grid-on'):
            plt.grid()

        if self.config.get('background', None):
            img = plt.imread(self.config['background'])
            img = np.flipud(img)
            plt.imshow(img, origin='lower', extent=limits.flatten(),
                       zorder=zorder['background'],
                       alpha=self.config['background-transparency'])

        # draw regions
        for r in wp.regions:
            r.style['zorder'] = zorder['global region']
            text = None
            if withtext:
                text = r.text
                r.textStyle['zorder'] = zorder['global region text']
            drawRegion2D(viewport, r, r.style, text, r.textStyle)

        # draw boundary
        drawBoundary2D(viewport, wp.boundary, wp.boundary.style)

        if show_robot:
            # draw robot
            r = 0 if expanded else self.robot.diameter/2
            drawRobot2D(viewport, self.robot, size=r, sensing_area=sensing_area,
                        zorder=zorder['robot'])

        if solution is not None:
            # drawGraph(viewport, self.offline.ts.g, zorder=zorder['global ts'],
            #           **self.config['global-ts-color'])
            drawPolicy(viewport, self.offline.ts.g, solution, zorder=zorder['global plan'],
                       color=self.config['global-policy-color'])
        elif self.offline is not None: # draw transition system
            drawGraph(viewport, self.offline.ts.g, zorder=zorder['global ts'],
                      **self.config['global-ts-color'])
