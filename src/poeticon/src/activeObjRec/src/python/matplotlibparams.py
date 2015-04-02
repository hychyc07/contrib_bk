#!/usr/bin/env python

import matplotlib

matplotlib.rcParams['figure.subplot.left']   = 0.1
matplotlib.rcParams['figure.subplot.bottom'] = 0.1    #0.125
matplotlib.rcParams['figure.subplot.right']  = 0.95
matplotlib.rcParams['figure.subplot.top']    = 0.9
#figWidthInches= 10 * 2  # --> wide
#aspectRatio = 0.8 / 2. # --> wide
#matplotlib.rcParams['figure.figsize'] = [figWidthInches, figWidthInches*aspectRatio]

matplotlib.rcParams['backend_fallback'] = False
matplotlib.rcParams['interactive'] = False
matplotlib.rcParams['lines.antialiased'] = True
matplotlib.rcParams['lines.linewidth'] = 1
matplotlib.rcParams['legend.fancybox'] = True
#matplotlib.rcParams['figure.dpi'] = 300
matplotlib.rcParams['figure.facecolor'] = 'white'
matplotlib.rcParams['figure.edgecolor'] = 'white'
matplotlib.rcParams['savefig.dpi'] = 300
matplotlib.rcParams['savefig.facecolor'] = 'white'
matplotlib.rcParams['savefig.edgecolor'] = 'white'
matplotlib.rcParams['savefig.extension'] = 'pdf'
matplotlib.rcParams['verbose.level'] = 'debug'
matplotlib.rcParams['verbose.fileo'] = 'sys.stdout'
#matplotlib.rcParams['text.usetex'] = True
#matplotlib.rcParams['font.family'] = 'sans-serif'
#matplotlib.rcParams['font.variant'] = 'normal'
matplotlib.rcParams['font.weight'] = 'medium'
matplotlib.rcParams['font.size']   = 12
matplotlib.rcParams['axes.titlesize']  = 'large'
matplotlib.rcParams['axes.labelsize']  = 'medium'
matplotlib.rcParams['xtick.labelsize'] = 'medium'
matplotlib.rcParams['ytick.labelsize'] = 'medium'
matplotlib.rcParams['legend.fontsize'] = 'large'

