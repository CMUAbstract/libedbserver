#!/usr/bin/python

import argparse
import sys
import pandas as pd
import pylab as pl

parser = argparse.ArgumentParser()
parser.add_argument('events_file',
    help='file with processed watchpoint data stream')
parser.add_argument('--out', '-o', 
    help='output file with the figure')
parser.add_argument('-w', '--width', type=float,
    help='width of figure (inches)')
parser.add_argument('-g', '--height', type=float,
    help='height of figure (inches)')
args = parser.parse_args()

d = pd.read_csv(args.events_file)

ENERGY_COLUMN = 'iter_delta_energy_prc'

# Drop invalid data points: energy cannot increase between consecutive watchpoints
d = d[d[ENERGY_COLUMN] <= 0]

# Flip sign: use 'energy cost' not 'energy difference': easier to interpret
d = -d

#fig = pl.gcf()
#fig.set_size_inches(args.width, args.height)
if args.width is not None and args.height is not None:
    figsize = (args.width, args.height)
else:
    figsize = None

#pl.rcParams.update({'font.size': 8})

fig = pl.figure(num=1, figsize=figsize, dpi=100)



ax = d[ENERGY_COLUMN].hist(cumulative=True, normed=1, histtype='step', bins=50, color='black')
ax.set_xlabel("Energy cost (% of max capacity)")
ax.set_ylabel("Cumulative Probability")

if args.out is not None:
    pl.savefig(args.out, bbox_inches='tight')

pl.show()
