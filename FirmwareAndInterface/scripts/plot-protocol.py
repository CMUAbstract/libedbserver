#!/usr/bin/python

import sys
import pandas as pd
import pylab as pl
import matplotlib.pyplot as plt
import numpy as np
import argparse

import env
from header_parser import Header

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

host_comm_header = Header(env.HOST_COMM_HEADER, enums=['RF_EVENT'])

EVENT_LABELS_MARGIN = 1.0

present_events = set(d['RF_EVENTS'].unique())
ordered_events = filter(lambda e: e in present_events, host_comm_header.enums['RF_EVENT'])
ordered_events = ordered_events[::-1] # top to bottom
event_idxs = {}
for idx, event in enumerate(ordered_events):
	event_idxs[event] = idx

def event_as_number(e):
	if type(e) is str:
		return event_idxs[e] + EVENT_LABELS_MARGIN
	else:
		return np.nan

rf_events_as_numbers = map(event_as_number, d['RF_EVENTS'])

plot_grid = (6, 1)
voltage_axes = plt.subplot2grid(plot_grid, (0, 0), rowspan=plot_grid[0] - 1)
rf_axes = plt.subplot2grid(plot_grid, (plot_grid[0] - 1, 0))

voltage_columns = filter(lambda c: c.startswith('V'), d.columns)
for voltage_column in voltage_columns:
	voltage_axes.plot(d[TIME_COLUMN], d[voltage_column], '-', label=voltage_column)

voltage_axes.grid(True)
voltage_axes.legend(loc=0)
voltage_axes.set_ylabel('Voltage (v)')
voltage_axes.set_ylim([0, 3.35])

rf_axes.plot(d[TIME_COLUMN], rf_events_as_numbers, '+')
rf_axes.grid(True, which="both")
rf_axes.set_yticks(np.arange(len(ordered_events)) + EVENT_LABELS_MARGIN)
rf_axes.set_yticklabels(ordered_events)
 
for tick in rf_axes.yaxis.get_major_ticks():
	tick.label.set_fontsize(8)

if len(voltage_columns) > 0:
	rf_axes.set_xlim(voltage_axes.get_xlim())
rf_axes.set_ylim([0, len(ordered_events) + EVENT_LABELS_MARGIN])

rf_axes.set_xlabel('Time (sec)')

plt.tight_layout()

if args.output is not None:
	pl.savefig(args.output, bbox_inches='tight')
else:
	pl.show()
