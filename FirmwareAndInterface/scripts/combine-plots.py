#!/usr/bin/python

import argparse
import pandas as pd
import pylab as pl
import numpy as np

from collections import OrderedDict

def comma_list(s):
    return s.split(",")
def float_comma_list(s):
    return map(float, s.split(","))

parser = argparse.ArgumentParser()
parser.add_argument('nw_data_file',
    help='file with the trace data for nw case')
parser.add_argument('ne_data_file',
    help='file with the trace data for ne case')
parser.add_argument('sw_data_file',
    help='file with the trace data for second nw case')
parser.add_argument('se_data_file',
    help='file with the trace data for se case')
parser.add_argument('--output', '-o',
    help='output figure file')
parser.add_argument('--channels', type=comma_list, default=['CH1', 'D1'],
    help='channels to include into the plot')
parser.add_argument('--labels', type=comma_list, default=['Vcap', 'Loop'],
    help='labels for the channels')
parser.add_argument('--x-range', type=float_comma_list,
    help='time range to include (in ms)')
parser.add_argument('--digital-offset', type=float, default=1.5,
    help='vertical position of the digital channels (in Volts)')
parser.add_argument('--digital-height', type=float, default=0.25,
    help='height of each digital channel (in Volts)')
parser.add_argument('--digital-margin', type=float, default=0.05,
    help='space between two digital channels (in Volts)')
parser.add_argument('--label-horizontal-margin', type=float, default=5,
    help='space between edge and label (in ms)')
parser.add_argument('--label-vertical-margin', type=float, default=0.05,
    help='space between the plotted line and the label (in V)')
args = parser.parse_args()

SCOPE_HEADER_ROWS = 20

Y_RANGE = [args.digital_offset - args.digital_margin, 2.8]

columns = ['TIME'] + args.channels

datasets = OrderedDict()

d = pd.read_csv(args.nw_data_file, skiprows=SCOPE_HEADER_ROWS)
datasets['nw'] = d[columns]

d = pd.read_csv(args.ne_data_file, skiprows=SCOPE_HEADER_ROWS)
datasets['ne'] = d[columns]

d = pd.read_csv(args.sw_data_file, skiprows=SCOPE_HEADER_ROWS)
datasets['sw'] = d[columns]

d = pd.read_csv(args.se_data_file, skiprows=SCOPE_HEADER_ROWS)
datasets['se'] = d[columns]

digital_channels = filter(lambda c: c.startswith('D'), args.channels)

for dpos, d in datasets.iteritems():
    height = args.digital_margin
    for idx, chan in enumerate(digital_channels):
        d[chan] = args.digital_offset + height + d[chan] * args.digital_height
        height += args.digital_height + args.digital_margin

if args.x_range is not None:
    for dpos, d in datasets.iteritems():
        d['TIME'] = (d['TIME'] + (-min(d['TIME']))) * 1000 # to positive ms
        datasets[dpos] = d[(args.x_range[0] <= d['TIME']) & (d['TIME'] <= args.x_range[1])]

fig, axes = pl.subplots(nrows=2, ncols=2)

x = 0
y = 0
for i, d in enumerate(datasets.values()):
    for j, chan in enumerate(args.channels):
        axes[x, y].plot(d['TIME'], d[chan], color='black')
    y += 1
    if y % 2 == 0:
        y = 0
        x += 1

# Place labels
x = 0
y = 0
for i, d in enumerate(datasets.values()):

    digital_height = args.digital_offset + args.digital_margin
    for idx, chan in enumerate(args.channels):
        if chan in digital_channels:
            label_x = d['TIME'].iloc[0] + args.label_horizontal_margin
            label_y = digital_height
            digital_height += args.digital_height + args.digital_margin
        else: # analog label right above the start of the waveform
            label_x = d['TIME'].iloc[0] + args.label_horizontal_margin
            label_y = d[chan].iloc[0] + args.label_vertical_margin
        axes[x, y].text(label_x, label_y, args.labels[idx], ha='left', va='bottom')
    y += 1
    if y % 2 == 0:
        y = 0
        x += 1



axes[0, 1].get_yaxis().set_visible(False)
axes[1, 1].get_yaxis().set_visible(False)

axes[0, 0].get_xaxis().set_visible(False)
axes[0, 1].get_xaxis().set_visible(False)

axes[0, 0].set_ylabel("Voltage (V)")
axes[1, 0].set_ylabel("Voltage (V)")

fig.text(0.5, 0.0075, 'Time (ms)', ha='center', va='center')
#fig.text(0.5, 0.5, 'Time (ms)', ha='center', va='center')

axes[0, 0].set_ylim(Y_RANGE)
axes[0, 1].set_ylim(Y_RANGE)
axes[1, 0].set_ylim(Y_RANGE)
axes[1, 1].set_ylim(Y_RANGE)

axes[0, 0].set_xticks(np.arange(datasets['nw']['TIME'].min(), datasets['nw']['TIME'].max(), 10.0))
axes[0, 1].set_xticks(np.arange(datasets['ne']['TIME'].min(), datasets['ne']['TIME'].max(), 10.0))
axes[1, 0].set_xticks(np.arange(datasets['sw']['TIME'].min(), datasets['sw']['TIME'].max(), 10.0))
axes[1, 1].set_xticks(np.arange(datasets['se']['TIME'].min(), datasets['se']['TIME'].max(), 10.0))

pl.tight_layout()

if args.output is not None:
    pl.savefig(args.output, bbox_inches='tight')
pl.show()
