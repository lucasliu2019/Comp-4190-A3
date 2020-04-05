__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import matplotlib.pyplot as plt
import numpy as np
import random


class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    # return rect of overlap
    def CaluculateOverlapRect(self, obs):
        # print('CalculateOverlap {0},{1}->{2},{3} with {4},{5}->{6},{7}'.format(self.x, self.y, self.x + self.width, self.y+self.height, obs.x, obs.y, obs.x + obs.width, obs.y + obs.height) )
        if (self.x < obs.x):
            left = obs.x
        else:
            left = self.x
        if ((self.x + self.width) < (obs.x + obs.width)):
            right = self.x + self.width
        else:
            right = obs.x + obs.width
        # print('CalculateOverlap max', max, 'min', min, 'overlapX', overlapX)
        if (self.y < obs.y):
            bot = obs.y
        else:
            bot = self.y
        if ((self.y + self.height) < (obs.y + obs.height)):
            top = self.y + self.height
        else:
            top = obs.y + obs.height
        # print('CalculateOverlap returns {0}'.format(overlap))
        return Rectangle(left,bot,right-left,top-bot)

    def CaluclateArea(self):
        return self.width*self.height

    def CalculateOverlap(self, obs):
        # print('CalculateOverlap {0},{1}->{2},{3} with {4},{5}->{6},{7}'.format(self.x, self.y, self.x + self.width, self.y+self.height, obs.x, obs.y, obs.x + obs.width, obs.y + obs.height) )
        if (self.x < obs.x):
            min = self.x
        else:
            min = obs.x
        if ((self.x + self.width) < (obs.x + obs.width)):
            max = obs.x + obs.width
        else:
            max = self.x + self.width
        overlapX = (max - min) - (self.width + obs.width)
        # print('CalculateOverlap max', max, 'min', min, 'overlapX', overlapX)
        if (self.y < obs.y):
            min = self.y
        else:
            min = obs.y
        if ((self.y + self.height) < (obs.y + obs.height)):
            max = obs.y + obs.height
        else:
            max = self.y + self.height
        overlapY = (max - min) - (self.height + obs.height)
        # print('CalculateOverlap max', max, 'min', min, 'overlapY', overlapY)
        if (overlapX < 0) and (overlapY < 0):
            overlap = overlapX * overlapY
        else:
            overlap = 0.0
        # print('CalculateOverlap returns {0}'.format(overlap))
        return overlap

    def InRect(self, pt):
        rtn = False
        if (pt[0] >= self.x) and (pt[0] <= (self.x + self.width)) and (pt[1] >= self.y) and (pt[1] <= (self.y + self.height)):
            rtn = True
        return rtn

    def __str__(self):
        return str(self.x) + ", " + str(self.y) + ", " + str(self.width) + ", " + str(self.height)


class Obstacle(Rectangle):
    def __init__(self, x, y, width, height, color=None):
        super().__init__(x, y, width, height)
        self.color = color
        if (color is not None):
            self.patch = plt.Rectangle((self.x, self.y), self.width, self.height, facecolor=color, edgecolor='#202020')


class PathPlanningProblem:
    def __init__(self, width, height, onum, owidth, oheight, owidth_min, oheight_min):
        self.width = width
        self.height = height
        self.obstacles = self.CreateObstacles(onum, owidth, oheight, owidth_min, oheight_min)

    # create a obstacle
    def CreateObstacles(self, onum, owidth, oheight, owidth_min, oheight_min):
        # store object
        obstacles = []
        # num of try limit to create object, prevent dead loop
        nTryLim = 1000
        # num of try to create object
        nTry = 0

        # garantee the size is down to min accuracy
        owidth_min=max(owidth_min, 1)
        oheight_min=max(oheight_min, 1)

        while ((len(obstacles) < onum) & (nTry <= nTryLim)):
            nTry = nTry + 1
            x = int(random.uniform(0.0, self.width))
            y = int(random.uniform(0.0, self.height))
            w = int(random.uniform(owidth_min, owidth))
            h = int(random.uniform(oheight_min, oheight))
            if (x + w) > self.width:
                w = self.width - x
                # make sure object width greater than min
                if w < owidth_min:
                    continue
            if (y + h) > self.height:
                h = self.height - y
                # make sure object height greater than min
                if w < oheight_min:
                    continue
            # create a colored object with location
            obs = Obstacle(x, y, w, h, '#808080')
            # overlap with other?
            found = False
            # loop through each object to find overlap
            # for o in obstacles:
            #    if ( o.CalculateOverlap(obs) > 0.0 ):
            #        found = True
            #        break
            # add when no overlap
            # if ( not found ):
            obstacles = obstacles + [obs]
        return obstacles

    # define the goal and initial point
    def CreateProblemInstance(self):
        found = False
        while (not found):
            ix = int(random.uniform(0.0, self.width))
            iy = int(random.uniform(0.0, self.height))

            oinitial = Obstacle(ix, iy, 1, 1)
            found = True
            for obs in self.obstacles:
                if (oinitial.CalculateOverlap(obs) > 0.0):
                    found = False
                    break

        found = False
        while (not found):
            gx = int(random.uniform(0.0, self.width))
            gy = int(random.uniform(0.0, self.height))

            ogoal = Obstacle(gx, gy, 1, 1)
            found = True
            for obs in self.obstacles:
                if (ogoal.CalculateOverlap(obs) > 0.0):
                    found = False
                    break
            if (oinitial.CalculateOverlap(ogoal) > 0.0):
                found = False

        return ((ix, iy), [(gx, gy)])

    def CheckOverlap(self, r):
        overlap = False
        for o in self.obstacles:
            if (r.CalculateOverlap(o) > 0):
                overlap = True
                break
        return overlap

    def CalculateCoverage(self, path, dim):
        x = np.arange(0.0, self.width, dim)
        y = np.arange(0.0, self.height, dim)
        counts = np.zeros((len(y), len(x)))
        for p in path:
            i = int(p[1] / dim)
            j = int(p[0] / dim)
            counts[j][i] = counts[j][i] + 1
        return (x, y, counts)
