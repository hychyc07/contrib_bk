from evaluation import *
import matplotlib.pyplot as plt

KNN = 'probs_knn'
PF  = 'probs'

def plotAccuracy(ax,data, modality, iteration, label=None, color=None, linestyle=None, marker=None):
    if linestyle == None:
        linestyle = '-'
    # calcute accuracy
    accs = []
    errs = []
    for res in data: 
        a, err = computeAccuracies(res[modality], res['objects'], res['truelabels'])
        accs.append(a[iteration])
        errs.append(err[iteration])
    # plot with error bars
    ax.errorbar(np.arange(len(accs))+0.5, [a*100 for a in accs], label=label, yerr=[e*100 for e in errs], \
            linestyle=linestyle, lw=2.0, marker=marker, color=color)
    

if __name__ == '__main__':

    iteration = 10
    dirs    = ['rotation_30neg', 'rotation_15neg', 'rotation_0', 'rotation_15', 'rotation_30', 'rotation_45', 'rotation_60']
    xlabels = ['-30', '-15', '0', '15', '30', '45', '60']


    assert(len(dirs) == len(xlabels))

    resultsPlanned        = []
    resultsRandom         = []
    resultsBoostedPlanned = []
    resultsBoostedRandom  = []

    # load data
    for rotDir in dirs:
        resultsPlanned.append(loadData(rotDir, [], 'results/planned'))
        resultsRandom.append(loadData(rotDir, [], 'results/random'))
        resultsBoostedPlanned.append(loadData(rotDir, [], 'results_boosted/planned'))
        resultsBoostedRandom.append(loadData(rotDir, [], 'results_boosted/random'))

    
    # create figure and plot data
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # plot each condition
    plotAccuracy(ax, resultsBoostedPlanned, PF,  iteration, label="Boosted Planned / Particle Filter", color='b', marker='s') 
    plotAccuracy(ax, resultsBoostedPlanned, KNN, iteration, label="Boosted Planned / KNN", color='r', marker='s') 
    plotAccuracy(ax, resultsBoostedRandom,  PF,  iteration, label="Boosted Random / Particle Filter", color='b', linestyle='--', marker='s')
    plotAccuracy(ax, resultsBoostedRandom,  KNN, iteration, label="Boosted Random / KNN", color='r', linestyle='--', marker='s') 
    plotAccuracy(ax, resultsPlanned, PF,  iteration, label="Planned / Particle Filter", color='b') 
    plotAccuracy(ax, resultsPlanned, KNN, iteration, label="Planned / KNN", color='r') 
    plotAccuracy(ax, resultsRandom,  PF,  iteration, label="Random / Particle Filter", color='b', linestyle='--')
    plotAccuracy(ax, resultsRandom,  KNN, iteration, label="Random / KNN", color='r', linestyle='--') 

    ax.set_xticks(np.arange(len(xlabels))+0.5)
    ax.set_xticklabels(xlabels)
    ax.set_ylim(0,100)
    ax.legend(loc='best')
    plt.show()
