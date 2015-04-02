#!/usr/bin/env python

from  __future__ import division
import os
import sys
import settings
import numpy as np
import matplotlib.pyplot as plt
from pylab import *

color=['#0000ff','#ff0000','#9999ff','#ff9999']

# load gaze files
def plotData(ax, resultSubDirs, explStrategy, caption):
    resultPath = os.path.join(settings.resultRoot,  'results_boxes/results_boosted', explStrategy)

    if (resultSubDirs == []):
        resultSubDirs = os.listdir(resultPath)

    resultSubDirs = [os.path.join(resultPath,d) for d in resultSubDirs]

    gaze = np.zeros((1, 2))
    for objdir in resultSubDirs:
        print objdir
        trialdirs = os.listdir(os.path.join(resultPath, objdir))
        gaze = np.zeros((len(trialdirs), 2))
        for i, trialdir in enumerate(trialdirs): 
            #try:
                gaze = np.loadtxt(os.path.join(objdir, trialdir, "gaze.txt"))
            #except:
                #print "Error loading gaze file" 
                #continue
                #size = ((np.arange(gaze.shape[0])+1)*4)**1.7
                size = ((np.arange(gaze.shape[0])+1)*4)**1.3
                ax.scatter(gaze[:,1], gaze[:,0], s=size,  alpha=0.5)

    
if __name__ == '__main__':   

    resultSubDirs = []
    # load data
    if len(sys.argv) > 1:
        plotObjName = sys.argv[1];
        resultSubDirs.append(plotObjName)
        
    objects = [
            'normcup',
            'rimcup',
            'bluecup',
            'insidecup',
            'redcup',
            'sidecup'
            ]
    objects = [
            'verde',
            'melissa',
            'fino',
            'frutti',
            ]
    #objects = [
            #'boxFennel',
            #'boxFruit',
            #'boxGreen',
            #'boxMint',
            #]

    captions = [
            'Normal cup',
            'Yellow/green inner border',
            'Blue side sticker',
            'Yellow/green inside bottom',
            'Red inside bottom',
            'Yellow/green side sticker'
            ]
    
    fig = plt.figure()
    #ax = fig.add_subplot(111)
    for i,obj in enumerate(objects):
        ax = fig.add_subplot(2,3,i+1)
        #plotData(ax, [obj], "planned", captions[i])
        ax.set_title(objects[i])
        plotData(ax, [obj], "planned", objects[i])
        ax.set_title(objects[i])
        ax.set_xlim(160, 280)
        ax.set_ylim(90, 20)
        ax.set_ylabel("Elevation (deg)")
        ax.set_xlabel("Azimuth (deg)")
    #plotData(["redCup", "insideCup"], "planned")
    #figure()
    #plotData(["blueCup"], "planned", "Blue)
    #figure()
    #plotData(["sideCup"], "planned", "Blue cup)


    plt.show()
