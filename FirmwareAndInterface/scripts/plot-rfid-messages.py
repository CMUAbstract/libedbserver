#!/usr/bin/python

import sys
import pandas as pd
import pylab as pl
import matplotlib.pyplot as plt
import numpy as np
import argparse

import env
from header_parser import Header

from plot_rfid_messages import *

def comma_list(s):
    return s.split(",")
def float_comma_list(s):
    return map(float, s.split(","))

parser = argparse.ArgumentParser()
parser.add_argument('data_file',
	help='file with voltage and protocol events in CSV format')
parser.add_argument('-r', '--range', nargs=2, metavar=['FROM', 'TO'],
	type=float, default=None,
	help='time range (in sec) to plot')
parser.add_argument('-o', '--output',
	help='filename to save figure to')
args = parser.parse_args()

TIME_COLUMN = 'timestamp_sec'

d = pd.read_csv(args.data_file)

if args.range is not None:
	d = d[(args.range[0] <= d[TIME_COLUMN]) & (d[TIME_COLUMN] <= args.range[1])]

plot_grid = (6, 1)

voltage_axes = plt.subplot2grid(plot_grid, (0, 0), rowspan=plot_grid[0] - 1)

if args.range is not None:
    d = d[(args.range[0] <= d[TIME_COLUMN]) & (d[TIME_COLUMN] <= args.range[1])]
time_values = d[TIME_COLUMN]
time_range = time_values.min(), time_values.max()

plot_voltage_trace(voltage_axes)

if 'RF_EVENTS' in d.columns:
    rf_axes = plt.subplot2grid(plot_grid, (plot_grid[0] - 1, 0))
    plot_rf_events(rf_axes)

plt.tight_layout()


if args.output is not None:
    pl.savefig(args.output, bbox_inches='tight')

pl.show()
