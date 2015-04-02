import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import glob


def getObjectname(filename):
    return os.path.split(filename)[1].split('.')[0].split('-')[0]
def getTrialNo(filename):
    return os.path.split(filename)[1].split('.')[0].split('-')[1]
def uniquify(l):
    return [x for i,x in enumerate(l) if x not in l[i+1:]]

def getAccuracy(m):
    confmat = m.copy()
    for i,row in enumerate(confmat):
        bestId = np.argmax(row)
        confmat[i,:] = 0
        confmat[i,bestId] = 1
    accuracy = np.sum(np.diag(confmat)) / confmat.shape[0] 
    return accuracy

def sumList(l):
    s = 0.0
    for v in l.split():
        s += float(v)
    return s

if __name__ == '__main__':

    resultDir = "../../data/vtm_results";

    resultFiles = sorted(glob.glob(os.path.join(resultDir,"*.txt"))) 
    trueLabels = [getObjectname(f) for f in resultFiles]
    objectNames = [n.strip() for n in open(os.path.join(resultDir,"../objects.txt"))]
    #objectNames = uniquify(trueLabels)
    #print trueLabels
    #print objectNames

    confmat = np.zeros((len(objectNames), len(objectNames)))

    objDict = {}
    for i,cl in enumerate(objectNames):
        objDict[i] = cl
    #print objDict
    
    nameDict = {}
    for i,l in enumerate(objectNames):
        nameDict[l] = i
    
    numTrials = np.zeros((len(objectNames),1));

    print nameDict

    # load results
    for objNo, objFilename in enumerate(resultFiles):
        resultsLists  = [l for l in open(objFilename)]
        probs = np.array([sumList(list) for list in resultsLists])
        if sum(probs) > 0:
            probs /= sum(probs)
        trueName = getObjectname(objFilename)#objDict[objNo]
        #trialNo = getTrialNo(objFilename)
        #if trueName == "box_w":
            #continue
        row = nameDict[trueName]
        print trueName

        numTrials[row] += 1

        for pNo,p in enumerate(probs):
            predName = objDict[pNo]
            #if predName == "box_w":
                #continue
            col = nameDict[predName]
            confmat[row, col] += p


    confmat /= numTrials

    print getAccuracy(confmat)

    fig = plt.figure()
    ax  = fig.add_subplot(111)
    img = ax.imshow(confmat, interpolation='nearest')
    ax.set_yticklabels(objectNames)
    ax.set_xticklabels(objectNames, rotation='90')

    ax.set_xticks(range(len(objectNames)))
    ax.set_yticks(range(len(objectNames)))

    fig.colorbar(img)
    plt.show()





