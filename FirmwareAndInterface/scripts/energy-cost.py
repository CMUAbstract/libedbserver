#!/usr/bin/python

import argparse
import sys
import pandas as pd
import pylab as pl
import numpy as np
import scikits.statsmodels.tools as sm

def comma_list(s):
    return s.split(",")

parser = argparse.ArgumentParser()
parser.add_argument('events_file', nargs='+',
    help='files with processed watchpoint data streams')
parser.add_argument('--out', '-o', 
    help='output file with the figure')
parser.add_argument('-w', '--width', type=float,
    help='width of figure (inches)')
parser.add_argument('-g', '--height', type=float,
    help='height of figure (inches)')
parser.add_argument('--labels', type=comma_list,
    help='labels for each CDF (comma-separated)')
parser.add_argument('--markers', type=comma_list,
    help='markers for each CDF (comma-separated)')
args = parser.parse_args()

ENERGY_COLUMN = 'iter_delta_energy_prc'

#fig = pl.gcf()
#fig.set_size_inches(args.width, args.height)
if args.width is not None and args.height is not None:
    figsize = (args.width, args.height)
else:
    figsize = None

if args.labels is not None and len(args.events_file) != len(args.labels):
    raise Exception("Number of labels does not correspond to number of datasets")

#pl.rcParams.update({'font.size': 8})

fig = pl.figure(num=1, figsize=figsize, dpi=100)

ax = pl.axes()

ax.set_xlabel("Energy cost (% of max capacity)")
ax.set_ylabel("Cumulative Probability")

energy_cost = {}

xmax = None

legend_handles = []

for i, file in enumerate(args.events_file):
    d = pd.read_csv(file)

    # Drop invalid data points
    # energy cannot increase between consecutive watchpoints
    # beyond 10% are outlines (valid values are within 6%)
    MAX_VALID_COST = 10
    d = d[(-MAX_VALID_COST <= d[ENERGY_COLUMN]) & (d[ENERGY_COLUMN] <= 0)]

    # Flip sign: use 'energy cost' not 'energy difference': easier to interpret
    d = -d

    s = d[ENERGY_COLUMN] 

    # manually define bins with a large upper boundary to avoid vertical line
    NUM_BINS = 50
    bins = list(np.arange(min(s), max(s), float(max(s) - min(s)) / NUM_BINS))
    bins.append(MAX_VALID_COST)

    if xmax is None or max(s) > xmax:
        xmax = max(s)

    if args.labels is not None:
        #legend_handles.append(pl.Line2D([], [], color='black',
        #                       marker=args.markers[i], label=args.labels[i]))
        label = args.labels[i]
    else:
        label = None

    ecdf = sm.tools.ECDF(s)
    x = np.linspace(min(s), max(s))
    y = ecdf(x)

    pl.step(x, y, color='black', label=label, marker=args.markers[i], markevery=5, markerfacecolor='gray')

    #pl.hist(s, cumulative=True, normed=1, histtype='step',
            #bins=bins, color='black', maker=args.markers[i])

pl.grid(True)

#ax.set_xlim(xmax=xmax)

if args.labels is not None:
    #pl.legend(handles=legend_handles, loc=0)
    pl.legend(loc=0, fontsize='small')

if args.out is not None:
    pl.savefig(args.out, bbox_inches='tight')

pl.show()
