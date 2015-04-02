#!/usr/bin/env python

from  __future__ import division
import numpy as np
import os
import settings
from collections import defaultdict
import scipy.stats.distributions as distributions

def loadOneTrial(path):
    filenameObjProbs    = os.path.join(path, "objprobs.txt") 
    filenameObjProbsKNN1 = os.path.join(path, "objprobs-knn-1.txt")
    filenameObjProbsKNN3 = os.path.join(path, "objprobs-knn.txt")
    filenameObjProbsKNN5 = os.path.join(path, "objprobs-knn-5.txt")
    filenameObjNames    = os.path.join(path, "objects.txt") 
    filenameTime        = os.path.join(path, "time.txt") 
   
    objProbs = np.loadtxt(filenameObjProbs)
    #objProbsKNN1 = np.loadtxt(filenameObjProbsKNN1)
    objProbsKNN3 = np.loadtxt(filenameObjProbsKNN3)
    #objProbsKNN5 = np.loadtxt(filenameObjProbsKNN5)
    objects = []
    times   = []
    with open(filenameObjNames) as f:
        for line in f:
            objects.append(line.rstrip())

    with open(filenameTime) as f:
        for line in f:
            times.append(line.rstrip())

    #return objProbs, [objProbsKNN1, objProbsKNN3, objProbsKNN5], objects, times
    return objProbs, [objProbsKNN3], objects, times
 

def loadData(resultDir, resultSubDirs, explStrategy):
    resultPath = os.path.join(settings.resultRoot, resultDir, explStrategy)

    if (resultSubDirs == []):
        resultSubDirs = os.listdir(resultPath)

    resultSubDirs = [os.path.join(resultPath, d) for d in resultSubDirs]
    #print resultSubDirs

    listObjProbs     = defaultdict(list)
    listObjProbsKNN1 = defaultdict(list)
    listObjProbsKNN3 = defaultdict(list)
    listObjProbsKNN5 = defaultdict(list)
    listObjects      = defaultdict(list) 
    listTrueLabel    = defaultdict(list) 
    listTimes        = []
    for objdir in resultSubDirs:
        print objdir
        for trialNo, trialdir in enumerate(os.listdir(os.path.join(resultPath, objdir))):
            #try:
            objProbs, objProbsKNN, objects, time = loadOneTrial(os.path.join(objdir, trialdir))
                #objProbs, objProbsKNN, objects = loadOneTrial(os.path.join(objdir, trialdir))
            #except:
                #continue
            if len(objProbs.shape) < 2 : continue
            if objProbs.shape == (): continue
            listObjProbs[trialNo].append(objProbs)
            #listObjProbsKNN1[trialNo].append(objProbsKNN[0])
            listObjProbsKNN3[trialNo].append(objProbsKNN[0])
            #listObjProbsKNN5[trialNo].append(objProbsKNN[2])
            listObjects[trialNo].append(objects)
            listTimes.append(time)
            objname = objdir.split('/')[-1]
            listTrueLabel[trialNo].append(objname)
        #a = computeAccuracies([objProbs], [objects], [truelabels])

    results = {}
    results['probs']       = listObjProbs; 
    results['probs_knn']   = listObjProbsKNN3; 
    results['probs_knn-1'] = listObjProbsKNN1; 
    results['probs_knn-5'] = listObjProbsKNN5; 
    results['objects']     = listObjects; 
    results['truelabels']  = listTrueLabel; 
    results['time']        = listTimes

    return results

def calcEntropy(probMat):
    probMat += 0.0001 # all values need to be >0.0 to compute entropy
    e = np.zeros((probMat.shape[0]))
    for i in np.arange(probMat.shape[0]):
        e[i] = distributions.entropy(probMat[i,:])
    return e

def computeAccuracies(listProbs, listObjects, listTrueLabel):
    winnerTakeAll = 1
    acc = []
    nIters = 15 
    for (trial, objProbs) in listProbs.items():
        nObjInDB = objProbs[0].shape[1]
        iterNPos = np.zeros((nIters));
        iterNNeg = np.zeros((nIters));
        nObjs = len(objProbs);
        confmat = np.zeros((nIters, nObjs, nObjInDB));
        for objNo, probs in enumerate(objProbs):
            for iteration in range(nIters):
                i = iteration
                if iteration >= probs.shape[0]:
                    i = probs.shape[0]-1
                #print probs[i,:]
                if winnerTakeAll:
                    bestId = np.argmax(probs[i,:])
                    #print bestId
                    #print listObjects[trial][objNo][bestId], listTrueLabel[trial][objNo]
                    if (listObjects[trial][objNo][bestId] == listTrueLabel[trial][objNo]):
                        iterNPos[iteration] += 1
                    else:
                        iterNNeg[iteration] += 1
                else:
                    for k,val in enumerate(probs[i,:]):
                        confmat[iteration,objNo,k] += val
        if winnerTakeAll:
            acc.append(iterNPos / (iterNPos+iterNNeg))
        else:
            a = np.zeros((nIters));
            for it in range(confmat.shape[0]):
                if nObjs == 1:
                    trueId = listObjects[trial][objNo].index(listTrueLabel[trial][objNo])
                    a[it] = confmat[it,objNo,trueId]
                else:
                    a[it] = np.sum(np.diag(np.reshape(confmat[it,:,:], (nObjs, nObjInDB)) )) / confmat.shape[1]
            acc.append(a)


    nTrials = len(acc)

    accmat = np.zeros((nTrials,nIters))
    for i,trial_accs in enumerate(acc):
        accmat[i,:] = trial_accs;
    
    A_stderr = np.std(accmat, axis=0)
    if nTrials > 0: 
        A_stderr /= np.sqrt(nTrials-1)

    acc = np.mean(accmat, axis=0)

    return acc, A_stderr


def mergeSingleObjectResults(listProbs):
    nresults = len(listProbs)
    evecList = []
    maxlen = 0
    for (trial, listProbs) in listProbs.items():
        for probs in listProbs:
            evec = calcEntropy(probs)
            evecList.append(evec)
            maxlen = max(maxlen, len(evec))

    nIters = 15
    jointEVec = np.zeros((maxlen, len(evecList)))
    for i, evec in enumerate(evecList):
        for j in range(evec.shape[0]):
            jointEVec[j, i] = evec[j]

    #E_stderr = np.zeros((maxlen))
    #E_mean = np.zeros((maxlen))
    E_stderr = np.zeros((nIters))
    E_mean = np.zeros((nIters))

    #for i in range(maxlen):
    for i in range(nIters):
        if i >= maxlen:
            i = maxlen-1
        vec = jointEVec[i,:]#np.nonzero(jointEVec[i,:])]
        #print vec
        E_mean[i] = np.mean(vec) 
        E_stderr[i] = np.std(vec)
        if len(vec) > 0:
            E_stderr[i] /= np.sqrt(len(vec)-1)

    return E_mean, E_stderr

def calcROC(listProbs, listObjects, listTrueLabels, iteration):
    #P = len(listTrueLabels[0])
    #if P == 0: 
        #return

    P = 0
    L = []

    for (trial, listProbs) in listProbs.items():
        for objNo, probs in enumerate(listProbs):
            i = iteration
            if iteration >= probs.shape[0]:
                i = probs.shape[0]-1
            P += 1
            for predId, pred in enumerate(probs[i,:]):
                example = {}
                example["name1"] = listObjects[trial][objNo][predId]
                example["name2"] = listTrueLabels[trial][objNo]
                example["score"] = pred
                L.append(example)
            
    L = sorted(L, key=itemgetter("score"), reverse=1)
    
    R = []   # ROC
    AUC = 0    
    FP = TP = 0
    FPprev = TPprev = 0
    N = len(L) - P
    Sprev = -1
    
    for example in L:
        if example["score"] != Sprev:
            AUC += trapezoid_area(FP, FPprev, TP, TPprev)
            R.append((FP/N, TP/P)) 
            Sprev = example["score"]
            FPprev = FP
            TPprev = TP
        if example["name1"] == example["name2"]:
            TP += 1
        else:
            FP += 1

    R.append((FP/N, TP/P)) 
    AUC += trapezoid_area(N, FPprev, P, TPprev)
    AUC /= P*N
    xaxis = [pnt[0] for pnt in R]
    yaxis = [pnt[1] for pnt in R]
    
    # find ERR
    EER = 1.0
    for idx, fp in enumerate(xaxis):
        if (1-yaxis[idx]) < fp:
            EER = fp
            break

    return xaxis, yaxis, AUC, EER 


def mergeTimes(listTimes):
    maxlen = 0
    for t in listTimes:
        maxlen = max(maxlen, len(t))

    #sumT = np.zeros((maxlen))
    #numTrialsIter = np.zeros((maxlen))
    #for trial, times in enumerate(listTimes):
        #for it, t in enumerate(times:
            #sumT[it] += t
            #numTrialsIter[it] += 1
    #sumT /= numTrialsIter

    ntrials = len(listTimes)
    jointTVec = np.zeros((maxlen, ntrials))
    for i, tvec in enumerate(listTimes):
        for j in range(len(tvec)):
            jointTVec[j, i] = tvec[j]

    T_stderr = np.zeros((maxlen))
    T_mean = np.zeros((maxlen))
    for i in range(maxlen):
        vec = jointTVec[i,np.nonzero(jointTVec[i,:])]
        #print vec
        T_mean[i] = np.mean(vec) 
        T_stderr[i] = np.std(vec) / np.sqrt(len(vec))

    return T_mean, T_stderr



