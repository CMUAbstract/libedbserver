#!/usr/bin/python

import argparse
import pandas as pd
import pylab as pl
import numpy as np

def comma_list(s):
    return s.split(",")

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

# Charging time. This value is approximate.
RIGHT_TIME_OFFSET = 0.0 # s

Y_RANGE = [args.digital_offset - args.digital_margin, 2.7]

columns = ['TIME'] + args.channels

d_nw = pd.read_csv(args.nw_data_file, skiprows=SCOPE_HEADER_ROWS)
d_nw = d_nw[columns]

d_ne = pd.read_csv(args.ne_data_file, skiprows=SCOPE_HEADER_ROWS)
d_ne = d_ne[columns]

d_sw = pd.read_csv(args.sw_data_file, skiprows=SCOPE_HEADER_ROWS)
d_sw = d_sw[columns]

d_se = pd.read_csv(args.se_data_file, skiprows=SCOPE_HEADER_ROWS)
d_se = d_se[columns]

datasets = [d_nw, d_ne, d_sw, d_se]

digital_channels = filter(lambda c: c.startswith('D'), args.channels)

for d in datasets:
    height = args.digital_margin
    for idx, chan in enumerate(digital_channels):
        d[chan] = args.digital_offset + height + d[chan] * args.digital_height
        height += args.digital_height + args.digital_margin

# Workaround for discrepancy in scope horizontal scale for the
# particular dataset
#d_nw = d_nw[(-0.005 <= d_nw['TIME']) & (d_nw['TIME'] <= 0.040)]
#d_ne = d_ne[(-0.01 <= d_ne['TIME']) & (d_ne['TIME'] <= 0.035)]
#
#d_sw = d_sw[(-0.005 <= d_sw['TIME']) & (d_sw['TIME'] <= 0.045)]
#d_se = d_se[(-0.01 <= d_se['TIME']) & (d_se['TIME'] <= 0.035)]
# TODO: bounds for nw2 + se

# to positive ms
d_time_nw = (d_nw['TIME'] + (-min(d_nw['TIME']))) * 1000
d_time_ne = (RIGHT_TIME_OFFSET + d_ne['TIME'] + (-min(d_ne['TIME']))) * 1000

d_time_sw = (d_sw['TIME'] + (-min(d_sw['TIME']))) * 1000
d_time_se = (RIGHT_TIME_OFFSET + d_se['TIME'] + (-min(d_se['TIME']))) * 1000

d_time = [d_time_nw, d_time_ne, d_time_sw, d_time_se]

fig, axes = pl.subplots(nrows=2, ncols=2)

x = 0
y = 0
for i, d in enumerate(datasets):
    for j, chan in enumerate(args.channels):
        axes[x, y].plot(d_time[i], d[chan], color='black')
    x += 1
    if x % 2 == 0:
        x = 0
        y += 1

# Place labels
x = 0
y = 0
for i, d in enumerate(datasets):

    digital_height = args.digital_offset + args.digital_margin
    for idx, chan in enumerate(args.channels):
        if chan in digital_channels:
            label_x = args.label_horizontal_margin
            label_y = digital_height
            digital_height += args.digital_height + args.digital_margin
        else: # analog label right above the start of the waveform
            label_x = args.label_horizontal_margin
            label_y = d[chan][0] + args.label_vertical_margin
        axes[x, y].text(label_x, label_y, args.labels[idx], ha='left', va='bottom')
    x += 1
    if x % 2 == 0:
        x = 0
        y += 1



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

axes[0, 0].set_xticks(np.arange(d_time_nw.min(), d_time_nw.max(), 10.0))
axes[0, 1].set_xticks(np.arange(d_time_ne.min(), d_time_ne.max(), 10.0))
axes[1, 0].set_xticks(np.arange(d_time_sw.min(), d_time_sw.max(), 10.0))
axes[1, 1].set_xticks(np.arange(d_time_se.min(), d_time_se.max(), 10.0))

pl.tight_layout()

if args.output is not None:
    pl.savefig(args.output, bbox_inches='tight')
pl.show()
