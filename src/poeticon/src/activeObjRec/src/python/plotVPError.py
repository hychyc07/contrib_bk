#!/usr/bin/env python

from  __future__ import division
import os
import sys
import settings
import numpy as np
import matplotlib.pyplot as plt

def loadErrors(resultDir, resultSubDirs, explStrategy, caption):
    resultPath = os.path.join(settings.resultRoot, resultDir, explStrategy)

    if (resultSubDirs == []):
        resultSubDirs = os.listdir(resultPath)

    resultSubDirs = [os.path.join(resultPath, d) for d in resultSubDirs]

    nObjects = len(resultSubDirs)
    objects_err_e = []
    objects_err_r = []

    nTrials = len(os.listdir(os.path.join(resultPath, resultSubDirs[0])))

    nIters = 15
    #assert(nTrials == 6)

    correct_trials_e = np.zeros((nTrials, nIters))
    correct_trials_r = np.zeros((nTrials, nIters))
    trialObjNo = np.zeros((nTrials,1))

    cnt = np.zeros((nObjects,nIters))
    for objdir in resultSubDirs:
        objectName = objdir.split('/')[-1] 
        print objectName 
        #print objdir
        trialdirs = os.listdir(os.path.join(resultPath, objdir))
        err_e = np.zeros((nTrials,nIters))
        err_r = np.zeros((nTrials,nIters))
        gaze = np.zeros((nTrials, 2))
        for i, trialdir in enumerate(trialdirs): 
            vp_r = []
            vp_e = []
            g_r = []
            g_e = []
            objProbs = np.loadtxt(os.path.join(objdir, trialdir, "objprobs.txt"))
            bestId = np.argmax(objProbs, axis=1)
            offset = 0

            # load object names
            objects = []
            filenameObjNames = os.path.join(objdir, trialdir, "objects.txt") 
            with open(filenameObjNames) as f:
                for line in f:
                    objects.append(line.rstrip())

            with open(os.path.join(objdir, trialdir, "viewpoints.txt")) as f:
                for lineno,line in enumerate(f):
                    vals = line.split();
                    e = float(vals[bestId[lineno]*2])
                    r = float(vals[bestId[lineno]*2+1])
                    vp_e.append(e)
                    vp_r.append(r)

            with open(os.path.join(objdir, trialdir, "gaze.txt")) as f:
                for lineno,line in enumerate(f):
                    e = float(line.split()[0])
                    r = float(line.split()[1])
                    offset_r = float(line.split()[2])
                    g_e.append(e)
                    g_r.append(r)
                    
            offset_e = 0
            d_e = np.array([abs((ge-offset_e)-ve) for (ve, ge) in zip(vp_e, g_e)])
            d_r = np.array([abs((gr-offset_r)-vr) for (vr, gr) in zip(vp_r, g_r)])
            err_e[i,:] = d_e
            err_r[i,:] = d_r

            # only keep if object was correctly recognized
            predObjectName = objects[bestId[-1]]
            correctDetection = predObjectName == objectName
            if correctDetection:
                trialObjNo[i] += 1
                correct_trials_r[i,:] += d_r
                correct_trials_e[i,:] += d_e

        objects_err_e.append(err_e)
        objects_err_r.append(err_r)

    # calc means and stds
    trial_e = np.zeros((nTrials, nIters))
    trial_r = np.zeros((nTrials, nIters))
    for objno, obj in enumerate(objects_err_e):
        trial_e += obj
    for objno, obj in enumerate(objects_err_r):
        trial_r += obj

    trial_e /= nObjects
    trial_r /= nObjects
    
    std_e = np.std(trial_e, axis=0)
    std_r = np.std(trial_r, axis=0)
    mean_e = np.mean(trial_e, axis=0)
    mean_r = np.mean(trial_r, axis=0)

    print trialObjNo
    #print trialObjNo==0
    trialObjNo[trialObjNo==0] = 1
    correct_trials_r /= trialObjNo
    correct_trials_e /= trialObjNo
    correct_trials_r = correct_trials_r[~np.all(correct_trials_r == 0, axis=1)]
    correct_trials_e = correct_trials_e[~np.all(correct_trials_e == 0, axis=1)]
    #correct_trials_r = np.ma.masked_invalid(correct_trials_r) # remove nans
    print correct_trials_r 
    std_r = np.std(correct_trials_r, axis=0)
    std_e = np.std(correct_trials_e, axis=0)
    mean_r = np.mean(correct_trials_r, axis=0)
    mean_e = np.mean(correct_trials_e, axis=0)

    return mean_e, mean_r, std_e, std_r

def plotError(ax, data, err, label):
    ax.errorbar(np.arange(0,data.shape[0]), data, label=label, lw=2.0, yerr=err)
    ax.set_ylabel("Avg. error (deg)")
    ax.set_xlabel("Iterations (deg)")


if __name__ == '__main__':   

    #if len(sys.argv) <= 1:
        #print "Usage: plotVPError.py resultDir [objectName]"
        #exit(0)

    # get result directory (fullpath = resultRoot + resultDir)
    #resultDir = sys.argv[1]
   
    resultSubDirs = []
    if len(sys.argv) > 2:
        plotObjName = sys.argv[2];
        resultSubDirs.append(plotObjName)


    fig = plt.figure()
    ax_e = fig.add_subplot(211)
    ax_r = fig.add_subplot(212)
    ax_e.set_ylim(0,90)
    ax_r.set_ylim(0,90)
    ax_e.set_title("Elevation")
    ax_r.set_title("Azimuth")

    # show change
    err_e, err_r, std_e, std_r = loadErrors("results_cups", resultSubDirs, "results_boosted/planned", [])
    plotError(ax_e, err_e, std_e, "-30 deg")
    plotError(ax_r, err_r, std_r, "-30 deg")

    #err_e, err_r, std_e, std_r = loadErrors("rotation_30neg", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "-30 deg")
    #plotError(ax_r, err_r, std_r, "-30 deg")
    #err_e, err_r, std_e, std_r = loadErrors("rotation_15neg", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "-15 deg")
    #plotError(ax_r, err_r, std_r, "-15 deg")
    #err_e, err_r, std_e, std_r = loadErrors("rotation_0", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "0 deg")
    #plotError(ax_r, err_r, std_r, "0 deg")
    #err_e, err_r, std_e, std_r = loadErrors("rotation_15", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "15 deg")
    #plotError(ax_r, err_r, std_r, "15 deg")
    #err_e, err_r, std_e, std_r = loadErrors("rotation_45", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "45 deg")
    #plotError(ax_r, err_r, std_r, "45 deg")
    #err_e, err_r, std_e, std_r = loadErrors("rotation_60", resultSubDirs, "results_boosted/planned", [])
    #plotError(ax_e, err_e, std_e, "60 deg")
    #plotError(ax_r, err_r, std_r, "60 deg")

    ax_e.legend(loc='best')
    ax_r.legend(loc='best')

    plt.show()

