import numpy as np
import settings
import matplotlib.pyplot as plt
import os

objectnames =[    "box_greentop"
                , "box_g_redbottom"
                , "box_w"
                , "box_green"
                , "box3"
                , "box_r_greenbottom"
                , "box_redgreen"
                , "box_yellowtop"
                , "box_blue"
                , "box_redtop"
                , "box_bluetop"
                , "box_redbottom"
                , "box_b_yellowbottom"
                , "box_greenside"
                , "box_y_whitebottom"
                , "box2"
                , "box_yellow"
                , "box_yellowbottom"
                , "box1"
                , "box_yellowblue"
                , "box_y_bluebottom"
                , "box_greenred"
                , "box_red"
                , "box_blueyellow"   ]
objectnames = [ "blueside"
                , "bs_ri"
                , "insideblue"
                , "insidered2"
                , "redside"
                , "rs_bi" ]

#filename = os.path.join(settings.resultRoot, "demo/results/random/bs_ri/01", "objprobs.txt")
filename = os.path.join(settings.resultRoot, "results_cups/results/planned/normcup/01", "objprobs.txt")
results = np.loadtxt(filename)

i = 0
for probs in results:
    fig = plt.figure()
    ax = fig.add_subplot(111)
    width = 0.8
    ind = np.arange(len(probs))
    ax.bar(ind+width/2, probs, width, color='k')
    ax.set_xticks(ind+width)
    ax.set_xticklabels(objectnames, rotation=45, size='small')
    ax.set_ylabel("Probability")
    ax.set_ylim(0,1)
    i+=1
    #plt.savefig("img/demo/random/"+str(i)+".pdf")
    plt.show(); 

