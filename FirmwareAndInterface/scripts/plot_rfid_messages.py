#!/usr/bin/python

import pandas as pd
import pylab as pl
import matplotlib.pyplot as plt
import numpy as np

import env
from header_parser import Header

TIME_COLUMN = 'timestamp_sec'

EVENT_LABELS_MARGIN = 1.0

host_comm_header = Header(env.HOST_COMM_HEADER, enums=['RF_EVENT'])

def hide_xaxis(axes):
    x_ticks = axes.get_xticks()
    axes.set_xticklabels([''] * len(x_ticks))

def plot_voltage_trace(voltage_axes, d, x_axis=False):
    #voltage_columns = filter(lambda c: c.startswith('V'), d.columns)
    voltage_columns = ['VCAP']
    d_voltages = d[[TIME_COLUMN] + voltage_columns].dropna()

    time_values = d[TIME_COLUMN]
    time_range = time_values.min(), time_values.max()

    for voltage_column in voltage_columns:
        voltage_axes.plot(d_voltages[TIME_COLUMN], d_voltages[voltage_column], '-',
                          label=voltage_column, color='black')

    voltage_axes.grid(True)
    #voltage_axes.legend(loc=0)
    voltage_axes.set_ylabel('Capacitor Voltage (v)')
    voltage_axes.set_ylim([1.5, 2.5])
    voltage_axes.set_xlim(time_range)

    if not x_axis:
        hide_xaxis(voltage_axes)


def plot_rf_events(rf_axes, d, x_axis=True, x_axis_label=True):
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

    time_values = d[TIME_COLUMN]
    time_range = time_values.min(), time_values.max()

    rf_axes.plot(d[TIME_COLUMN], rf_events_as_numbers, '+', color='black')

    rf_axes.set_yticks(np.arange(len(ordered_events)) + EVENT_LABELS_MARGIN)
    rf_axes.set_yticklabels(ordered_events)

    # This hides the vertical grid lines, so do a workaround
    #rf_axes.get_xaxis().set_visible(x_axis)
    if not x_axis:
        hide_xaxis(rf_axes)

    rf_axes.grid(True, which="both")

    for tick in rf_axes.yaxis.get_major_ticks():
        tick.label.set_fontsize(8)

    rf_axes.set_xlim(time_range)
    rf_axes.set_ylim([0, len(ordered_events) + EVENT_LABELS_MARGIN])

    if x_axis and x_axis_label:
        rf_axes.set_xlabel('Time (sec)')
