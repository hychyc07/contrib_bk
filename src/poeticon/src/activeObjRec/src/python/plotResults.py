#!/usr/bin/env python

from  __future__ import division
import os
import sys
import settings
import numpy as np
import matplotlib.pyplot as plt
import matplotlibparams
from collections import defaultdict
from operator import itemgetter
from evaluation import *

KNN = 'probs_knn'
PF  = 'probs'
PLANNED = True
RANDOM = False
BOOSTED = True

numIterations = 10 

def definePlotSettings(modality, planning, boosting, label, color, linestyle, lineweight, marker):
    str_modality = "KNN" if modality == KNN else "Particle Filter"
    str_planning = "Planned" if planning else "Random"
    str_boosting = "(Boosted)" if boosting else ""

    if linestyle is None: linestyle = '-' if modality == PF else '--'
    if marker    is None: marker    = 's' if boosting else None
    if label     is None: label     = str_planning + " " + str_modality + " " + str_boosting;
    if lineweight is None: lineweight = 2.0
    if color     is None: color     ='#3150CC' if planning else '#F0941D'

    return label, color, linestyle, lineweight, marker
    
def plotEntropy(ax, data, modality, planning=False, boosting=False, label=None, color=None, linestyle=None, marker=None, lineweight=None):
    label, color, linestyle, lineweight, marker = definePlotSettings(modality, planning, boosting, label, color, linestyle, linestyle, marker)
    e, e_err = mergeSingleObjectResults(data[modality])
    ax.errorbar(np.arange(0,e.shape[0])+1, e,label=label, color=color, yerr=e_err,  linestyle=linestyle, lw=lineweight, marker=marker)

def plotAccuracy(ax, data, modality, planning=False, boosting=False, label=None, color=None, linestyle=None, marker=None, lineweight=None):
    label, color, linestyle, lineweight, marker = definePlotSettings(modality, planning, boosting, label, color, linestyle, lineweight, marker)
    a, stderr = computeAccuracies(data[modality], data['objects'], data['truelabels'])
    ax.errorbar(np.arange(0,a.shape[0])+1, a*100, label=label, color=color, yerr=stderr*100, linestyle=linestyle, lw=lineweight, marker=marker)

def plotEntropyOverTime(ax,listProbs, listTimes, label, color=None, linestyle=None):
    if linestyle is None:
        linestyle = '-'
    t, t_err = mergeTimes(listTimes)
    e, e_err = mergeSingleObjectResults(listProbs)
    ax.errorbar(t, e,label=label, color=color, yerr=e_err,  linestyle=linestyle)


def plotROC(ax, listProbs, listObjects, listTrueLabel, label, color=None, linestyle=None):
    if linestyle is None:
        linestyle = '-'
    aucList = []
    nIters = 15
    for i in range(0,nIters):
        xaxis, yaxis, auc, eer = calcROC(listProbs, listObjects, listTrueLabel, iteration=i)
        aucList.append(auc)

    ax.plot(np.arange(0,len(aucList)), aucList, label=label, color=color, linestyle=linestyle, lw=2.0)
    #ax.plot(xaxis, yaxis, label=label, color=color, linestyle=linestyle, lw=2.0)

def trapezoid_area(x1, x2, y1, y2):
    base = abs(x1 - x2)
    height = (y1 + y2)/2
    return base * height


def iterAcc(results, matchType):
    return computeAccuracies(results[matchType], results['objects'], results['truelabels'])


def createBars(iterToShow):
    fig = plt.figure()

    objects = [
            'normCup',
            'borderCup',
            'blueCup',
            'insideCup',
            'redCup',
            'sideCup'
            ]

    captions = [
            'Normal cup',
            'Yellow/green inner border',
            'Blue side sticker',
            'Yellow/green inside bottom',
            'Red inside bottom',
            'Yellow/green side sticker'
            ]
            

    for n, obj in enumerate(objects):
        ax = fig.add_subplot(2,3,n+1)
        createBarsSubplot(ax, obj, iterToShow, captions[n]);



def createBarsSubplot(ax, objectName, iterToShow, caption):
    resultsPlanned = loadData([objectName], "planned")
    resultsRandom  = loadData([objectName], "random")

    nConditions = 4
    nIterations = 15
    accConditions = np.zeros((nConditions, nIterations))
    accConditions[0,:] = iterAcc(resultsPlanned, 'probs')
    accConditions[1,:] = iterAcc(resultsPlanned, 'probs_knn')
    accConditions[2,:] = iterAcc(resultsRandom, 'probs')
    accConditions[3,:] = iterAcc(resultsRandom, 'probs_knn')
    print accConditions[3,:]
    accConditions *= 100
    accConditions += 0.01 # to make sure we get a bar even if the value is 0
    offset = 0.0
    width = 0.8
    ind = np.arange(nConditions)
    ax.bar(ind+width/2, accConditions[:, iterToShow], width, color=['#0000ff','#ff0000','#9999ff','#ff9999'])  
    ax.set_ylim(0.0, 100.0)
    ax.set_ylabel('Accuracy (%)')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(('Planned\nPF', 'Planned\nKNN', 'Random\nPF', 'Random\nKNN'))
    ax.set_title(caption)

if __name__ == '__main__':   

    #iterToShow = 9 
    #createBars(iterToShow)
    #iterToShow = 14 
    #createBars(iterToShow)

    if len(sys.argv) <= 1:
        print "Usage: plotResults.py resultDir [objectName]"
        exit(0)

    # get result directory (fullpath = resultRoot + resultDir)
    resultDir = sys.argv[1]
    
    resultSubDirs = []
    if len(sys.argv) > 2:
        plotObjName = sys.argv[2];
        resultSubDirs.append(plotObjName)

    # load data
    resultsPlanned = loadData(resultDir, resultSubDirs, "results/planned")
    resultsRandom  = loadData(resultDir, resultSubDirs, "results/random")
    try:
        resultsBoostedPlanned  = loadData(resultDir, resultSubDirs, "results_boosted/planned")
    except:
        resultsBoostedPlanned   = None
    try:
        resultsBoostedRandom   = loadData(resultDir, resultSubDirs, "results_boosted/random")
    except:
        resultsBoostedRandom   = None

    # Plot ROC curve
    if False:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plotROC(ax, resultsPlanned['probs_knn'], resultsPlanned['objects'], resultsPlanned['truelabels'], label="Planned / KNN", color='r');
        plotROC(ax, resultsPlanned['probs'],     resultsPlanned['objects'], resultsPlanned['truelabels'], label="Planned / Particle Filter", color='b');
        plotROC(ax, resultsBoostedPlanned['probs'], resultsBoostedPlanned['objects'], resultsBoostedPlanned['truelabels'], label="Boosted Planned / Particle Filter", color='k');
        ax.set_ylim(0, 1.0)
        ax.set_ylabel("AUC")
        ax.set_xlabel("Interations")
        ax.legend(loc='best')
        ax.set_title("AUC")


    fig = plt.figure()
    #fig.suptitle(resultDir)
    #figWidthInches = 10 * 2  # --> wide
    #aspectRatio = 0.8 / 2. # --> wide
    #fig.set_size_inches(figWidthInches,figWidthInches*aspectRatio)

    # Plot avg accuracy over iterations
    ax = fig.add_subplot(111)
    plotAccuracy(ax, resultsPlanned,         PF,  planning=True,  boosting=False);
    plotAccuracy(ax, resultsPlanned,         KNN, planning=True,  boosting=False);
    if resultsBoostedPlanned is not None:
        plotAccuracy(ax, resultsBoostedPlanned,  PF,  planning=True,  boosting=True, lineweight=3.0);
        plotAccuracy(ax, resultsBoostedPlanned,  KNN, planning=True,  boosting=True);
    if resultsBoostedRandom is not None:
        plotAccuracy(ax, resultsBoostedRandom,   PF,  planning=False, boosting=True);
        plotAccuracy(ax, resultsBoostedRandom,   KNN, planning=False, boosting=True);
    if resultsRandom is not None:
        plotAccuracy(ax, resultsRandom,          PF,  planning=False, boosting=False);
        plotAccuracy(ax, resultsRandom,          KNN, planning=False, boosting=False);
    ax.set_xlim(1, numIterations)
    ax.set_ylim(0, 100)
    ax.set_ylabel("Accuracy (%)")
    ax.set_xlabel("Iterations")
    ax.legend(loc='best')
    ax.set_title("Accuracy")
    #plt.savefig("img/"+resultDir+"_A.pdf")

    #Plot entropy over iterations
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plotEntropy(ax, resultsPlanned,         PF,  planning=True,  boosting=False);
    plotEntropy(ax, resultsPlanned,         KNN, planning=True,  boosting=False);
    if resultsBoostedPlanned is not None:
        plotEntropy(ax, resultsBoostedPlanned,  PF,  planning=True,  boosting=True, lineweight=3.0);
        plotEntropy(ax, resultsBoostedPlanned,  KNN, planning=True,  boosting=True);
    if resultsBoostedRandom is not None:
        plotEntropy(ax, resultsBoostedRandom,   PF,  planning=False, boosting=True);
        plotEntropy(ax, resultsBoostedRandom,   KNN, planning=False, boosting=True);
    if resultsRandom is not None:
        plotEntropy(ax, resultsRandom,          PF,  planning=False, boosting=False);
        plotEntropy(ax, resultsRandom,          KNN, planning=False, boosting=False);
    ax.set_xlim(1, numIterations)
    #ax.set_ylim(0, 4.0)
    ax.set_ylabel("Entropy")
    ax.set_xlabel("Interations")
    ax.legend(loc='best')
    ax.set_title("Entropy")

    #plt.savefig("img/"+resultDir+"_H.pdf")

    plt.show()
