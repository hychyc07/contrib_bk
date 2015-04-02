import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import glob
from evaluation import *

def getAccuracy(m):
    confmat = m.copy()
    #for i,row in enumerate(confmat):
        #bestId = np.argmax(row)
        #confmat[i,:] = 0
        #confmat[i,bestId] = 1
    accuracy = np.sum(np.diag(confmat)) / confmat.shape[0] 
    return accuracy

if __name__ == '__main__':

    if len(sys.argv) <= 1:
        print "Usage: plotResults.py resultDir"
        exit(0)

    # get result directory (fullpath = resultRoot + resultDir)
    resultDir = sys.argv[1]

    iteration = 2
    if len(sys.argv) > 2:
        iteration = int(sys.argv[2])

    print iteration
    modality = 'probs_knn'
    #modality = 'probs'

    # load data
    #results = loadData(resultDir, [], 'results/random')
    results = loadData(resultDir, [], 'results/planned')

    listProbs = results[modality]
    objects = results['objects'][0][0]
    trueLabels = results['truelabels'][0]
    
    print objects
    objDict = {}
    for i,cl in enumerate(objects):
        objDict[i] = cl

    nameDict = {}
    for i,l in enumerate(trueLabels):
        nameDict[l] = i

    print nameDict
    confmat = np.zeros((len(trueLabels), len(trueLabels)))

    for (trial, objProbs) in listProbs.items(): 
        for objNo, probs in enumerate(objProbs):
            #for iteration in range(15):
                i = iteration
                if iteration >= probs.shape[0]:
                    i = probs.shape[0]-1
                row = objNo
                #for k,val in enumerate(probs[i,:]):
                    #predName = objDict[k]
                    #col = nameDict[predName]
                    #confmat[row, col] += val
                #print probs[i,:]
                confmat[row, np.argmax(probs[i,:])] += 1 

    confmat /= len(listProbs)
    print getAccuracy(confmat)

    fig = plt.figure()
    ax  = fig.add_subplot(111)
    img = ax.imshow(confmat, interpolation='nearest', vmax=1.0, vmin=0.0)
    ax.set_yticklabels(trueLabels)
    ax.set_xticklabels(trueLabels, rotation='90')

    ax.set_xticks(range(len(trueLabels)))
    ax.set_yticks(range(len(trueLabels)))

    fig.colorbar(img)
    plt.show()
