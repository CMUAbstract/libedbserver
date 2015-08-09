#!/usr/bin/python

import argparse
import pandas as pd
import pylab as pl
import numpy as np
from matplotlib import lines

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
parser.add_argument('-w', '--width', type=float,
    help='width of figure (inches)')
parser.add_argument('-g', '--height', type=float,
    help='height of figure (inches)')
parser.add_argument('--channels', type=comma_list, default=['CH1', 'D1'],
    help='channels to include into the plot')
parser.add_argument('--labels', type=comma_list, default=['Vcap', 'Loop'],
    help='labels for the channels')
parser.add_argument('--channel-format', type=comma_list, default=['analog', 'digital'],
    help='format in which to draw the channel data (analog,digital,digital-edges)')
parser.add_argument('--vcap-channel', default='CH1',
    help='channels with Vcap data (for cleaning up noise on digital channels)')
parser.add_argument('--digital-noise-threshold', type=float, default=1.82,
    help='threshold on Vcap beyond which data on digital channels is invalid')
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
parser.add_argument('--assert-plot-workaround', action='store_true',
    help='workaround for the assert null-deref use case: ' +
    'inconsistent time scale for the assert plot ' +
    '(cannot completely fix it in data)')
args = parser.parse_args()

SCOPE_HEADER_ROWS = 20
Y_RANGE_MAX = 2.8

Y_RANGE = [args.digital_offset - args.digital_margin, Y_RANGE_MAX]

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

analog_channels = filter(lambda c: c.startswith('CH'), args.channels)
digital_channels = filter(lambda c: c.startswith('D'), args.channels)

# remove noise from digital channels that starts happening
# once we get close to the digital threshold (couldn't get
# the scope to eliminate it by raising the threshold).
if len(digital_channels) > 0:
    for dpos, d in datasets.iteritems():
        vcap_chan = d[args.vcap_channel]
        for idx, chan in enumerate(digital_channels):
            d[chan][vcap_chan < args.digital_noise_threshold] = 0

for dpos, d in datasets.iteritems():
    d['TIME'] = (d['TIME'] + (-min(d['TIME']))) * 1000 # to positive ms


if args.assert_plot_workaround:
    datasets['se']['TIME'] += 20

if args.x_range is not None:
    for dpos, d in datasets.iteritems():
        datasets[dpos] = d[(args.x_range[0] <= d['TIME']) & (d['TIME'] <= args.x_range[1])]


fig, axes = pl.subplots(nrows=2, ncols=2)

if args.width is not None and args.height is not None:
    figsize = (args.width, args.height)
else:
    figsize = None
pl.figure(num=1, figsize=figsize, dpi=100)

x = 0
y = 0
for i, d in enumerate(datasets.values()):

    ax = axes[x, y]
    ax.tick_params(right=False)

    if y == 0:
        ax.spines['right'].set_visible(False)
    else:
        ax.spines['left'].set_visible(False)

    digital_height = args.digital_margin + args.digital_offset
    for j, chan in enumerate(args.channels):
        chan_data = d[chan].copy()

        ax.axhline(digital_height, 0, 1, linestyle='dotted', color='gray')

        chan_format = args.channel_format[j]

        if chan_format == "analog":
            ax.plot(d['TIME'], chan_data, color='black')

        elif chan_format == "digital":
            
            # move to the right position in the plot and mask out non-edges (need float's nan for that)
            digital_data_display = chan_data.copy() * 1.0
            digital_data_display[chan_data == 1] = digital_height
            digital_data_display[chan_data == 0] = float('nan')

            ax.plot(d['TIME'], digital_data_display, color='black', lw=4)

            digital_height += args.digital_height + args.digital_margin

        elif chan_format == "digital-edges":
            # detect edges
            time_array = d['TIME'][0:-1]
            digital_data_prev = chan_data.iloc[0:-1]
            digital_data_next = chan_data.iloc[1:]
            digital_data_prev.index = digital_data_next.index = \
                    time_array.index = range(len(time_array))

            # get ones for edges and zeroes for non-edges
            digital_data_edges = digital_data_prev.copy()
            digital_data_edges[((digital_data_prev != 0) | (digital_data_next != 1))] = 0
            digital_data_edges[((digital_data_prev == 0) & (digital_data_next == 1))] = 1

            # filter to include every k'th edge
            INCLUDE_EVERY = 2
            digital_data_cumsum = pd.Series(np.cumsum(digital_data_edges))
            digital_data_edges[digital_data_cumsum % INCLUDE_EVERY != 0] = 0

            # move to the right position in the plot and mask out non-edges (need float's nan for that)
            digital_data_display = digital_data_edges.copy() * 1.0
            digital_data_display[digital_data_edges == 1] = digital_height
            digital_data_display[digital_data_edges == 0] = float('nan')

            ax.scatter(time_array, digital_data_display, color='black', marker='o')

            digital_height += args.digital_height + args.digital_margin

    ax.set_xlim(args.x_range)

    if y == 1:
        xticks = ax.get_xticks()
        print xticks
        xtick_labels = ax.get_xticklabels()
        xtick_labels_editted = []
        print xtick_labels
        for label in xtick_labels:
            #print label
            xtick_labels_editted.append(label.get_text())
        print  xtick_labels_editted
        ax.set_xticklabels(map(lambda t: '+' + str(int(t)), xticks))

    y += 1
    if y % 2 == 0:
        y = 0
        x += 1

# Place labels
x = 0
y = 0
for i, d in enumerate(datasets.values()):

    # Don't label the 'west' side, to show continuity
    if y == 0:
        digital_height = args.digital_offset + args.digital_margin
        for idx, chan in enumerate(args.channels):
            if chan in digital_channels:
                label_x = d['TIME'].iloc[0] + args.label_horizontal_margin
                print "dig_height=", digital_height, "time=", label_x
                label_y = digital_height + args.label_vertical_margin
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

if args.output is not None:
    pl.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0.15, hspace=0.05)
else:
    pl.subplots_adjust(wspace=0.15, hspace=0.05)

fig.text(0.5, -0.075, 'Time (ms)', ha='center', va='bottom')

ax_overlay = pl.axes([0, 0, 1, 1], axisbg=(0, 0, 0, 0))
ax_overlay.axis('off')

for x in [0, 1]:
    for left_end, right_end in [((1,1), (0, 1)), ((1, 0), (0, 0))]:
        bridge_end_left = \
                ax_overlay.transAxes.inverted().transform(axes[x, 0].transAxes.transform(left_end))
        bridge_end_right = \
                ax_overlay.transAxes.inverted().transform(axes[x, 1].transAxes.transform(right_end))

        line_x, line_y = np.array([[bridge_end_left[0], bridge_end_right[0]],
                                   [bridge_end_left[1], bridge_end_right[1]]])
        line = lines.Line2D(line_x, line_y, color='black')
        ax_overlay.add_line(line)

    # '...' text
    DOTS_Y_POS = 0.6
    left_edge = ax_overlay.transAxes.inverted().transform(axes[x, 0].transAxes.transform((1, DOTS_Y_POS)))
    right_edge = ax_overlay.transAxes.inverted().transform(axes[x, 1].transAxes.transform((0, DOTS_Y_POS)))
    text_pos = [(float(left_edge[0] + right_edge[0]) / 2), left_edge[1]]
    ax_overlay.text(text_pos[0], text_pos[1], '...', ha='center', va='center', fontsize='24')


axes[0, 0].set_ylim(Y_RANGE)
axes[0, 1].set_ylim(Y_RANGE)
axes[1, 0].set_ylim(Y_RANGE)
axes[1, 1].set_ylim(Y_RANGE)

#axes[0, 0].set_xticks(np.arange(datasets['nw']['TIME'].min(), datasets['nw']['TIME'].max(), 10.0))
#axes[0, 1].set_xticks(np.arange(datasets['ne']['TIME'].min(), datasets['ne']['TIME'].max(), 10.0))
#axes[1, 0].set_xticks(np.arange(datasets['sw']['TIME'].min(), datasets['sw']['TIME'].max(), 10.0))
#axes[1, 1].set_xticks(np.arange(datasets['se']['TIME'].min(), datasets['se']['TIME'].max(), 10.0))

#pl.tight_layout()

if args.output is not None:
    pl.savefig(args.output, bbox_inches='tight')
pl.show()
