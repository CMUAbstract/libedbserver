#!/usr/bin/python

import sys
import pandas as pd
import pylab as pl
import matplotlib.pyplot as plt
import numpy as np
import argparse

from matplotlib import lines

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
parser.add_argument('-z', '--zoomed-range', nargs=2, metavar=['FROM', 'TO'],
	type=float, default=None,
	help='time range (in sec) for the zoomed in plot')
parser.add_argument('-o', '--output',
	help='filename to save figure to')
args = parser.parse_args()

TIME_COLUMN = 'timestamp_sec'

d = pd.read_csv(args.data_file)

d_full = d[(args.range[0] <= d[TIME_COLUMN]) & (d[TIME_COLUMN] <= args.range[1])]


plot_grid = (4, 1)

voltage_axes = plt.subplot2grid(plot_grid, (0, 0), rowspan=plot_grid[0] - 2)
plot_voltage_trace(voltage_axes, d_full)

rf_axes = plt.subplot2grid(plot_grid, (plot_grid[0] - 2, 0))
plot_rf_events(rf_axes, d_full, x_axis=True, x_axis_label=False)

d_zoomed = d[(args.zoomed_range[0] <= d[TIME_COLUMN]) & (d[TIME_COLUMN] <= args.zoomed_range[1])]

zoomed_rf_axes = plt.subplot2grid(plot_grid, (plot_grid[0] - 1, 0))
plot_rf_events(zoomed_rf_axes, d_zoomed)

center_x_data = (args.zoomed_range[1] + args.zoomed_range[0]) / 2

ZOOM_ANNOT_LINESTYLE = 'dashed'
ZOOM_ANNOT_LINECOLOR = 'gray'
ZOOM_ANNOT_LINEWEIGHT = 1.0

zoom_range = args.zoomed_range[1] - args.zoomed_range[0]

#zoom_box_width = zoom_range
#zoom_box_height = 1.0 # TODO: how to get this in data coords
#zoom_box = pl.Rectangle((center_x_data, 1.5), zoom_box_width, zoom_box_height,
#                        fill=False)
#rf_axes.add_patch(zoom_box)
for xpos in args.zoomed_range:
    rf_axes.axvline(xpos, color=ZOOM_ANNOT_LINECOLOR, lw=ZOOM_ANNOT_LINEWEIGHT,
                    linestyle=ZOOM_ANNOT_LINESTYLE)

#fig = pl.gcf()

# new clear axis overlay with 0-1 limits
zoomed_annot_axes = pl.axes([0,0,1,1], axisbg=(1,1,1,0))
#zoomed_annot_axes.get_xaxis().set_visible(False)
#zoomed_annot_axes.get_yaxis().set_visible(False)
#zoomed_annot_axes.patch.set_visible(False)
zoomed_annot_axes.axis('off')

zoomed_ax_transform = zoomed_rf_axes.transAxes
full_data_transform = rf_axes.transData
full_ax_transform = rf_axes.transAxes
zoomed_annot_transform = zoomed_annot_axes.transAxes.inverted()

top_left_display = zoomed_ax_transform.transform((0, 1))
top_right_display = zoomed_ax_transform.transform((1, 1))
top_left_annot = zoomed_annot_transform.transform(top_left_display)
top_right_annot = zoomed_annot_transform.transform(top_right_display)

left_center_x_data = center_x_data - zoom_range / 2
right_center_x_data = center_x_data + zoom_range /2

center_y_display = full_ax_transform.transform((0, 0.5))

left_center_x_display = full_data_transform.transform((left_center_x_data, 0))
left_center_ax_annot = zoomed_annot_transform.transform((left_center_x_display[0], center_y_display[1]))

right_center_x_display = full_data_transform.transform((right_center_x_data, 0))
right_center_ax_annot = zoomed_annot_transform.transform((right_center_x_display[0], center_y_display[1]))

line_x_array, line_y_array = np.array([[top_left_annot[0], left_center_ax_annot[0]],
                                       [top_left_annot[1], left_center_ax_annot[1]]])
left_line = lines.Line2D(line_x_array, line_y_array,
                         color=ZOOM_ANNOT_LINECOLOR, lw=ZOOM_ANNOT_LINEWEIGHT,
                         linestyle=ZOOM_ANNOT_LINESTYLE)


line_x_array, line_y_array = np.array([[right_center_ax_annot[0], top_right_annot[0]],
                                       [right_center_ax_annot[1], top_right_annot[1]]])
right_line = lines.Line2D(line_x_array, line_y_array,
                         color=ZOOM_ANNOT_LINECOLOR, lw=ZOOM_ANNOT_LINEWEIGHT,
                         linestyle=ZOOM_ANNOT_LINESTYLE)

zoomed_annot_axes.add_line(left_line)
zoomed_annot_axes.add_line(right_line)

#plt.tight_layout()
#plt.subplots_adjust(hspace=0.25) # to give some space between full and zoomed rf axes

if args.output is not None:
    pl.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0.15, hspace=0.2)
else:
    pl.subplots_adjust(wspace=0.15, hspace=0.2) # give some space between full and zoomed rf axes

if args.output is not None:
    pl.savefig(args.output, bbox_inches='tight')

pl.show()
