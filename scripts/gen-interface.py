#!/usr/bin/python

import os
import json
import argparse

from header_parser import Header

parser = argparse.ArgumentParser(
            description="Generate a file defining the interface to EDB (e.g. for pyedb)")
parser.add_argument('out',
            help="Output file with the interface data (JSON)")
args = parser.parse_args()

script_dir = os.path.dirname(os.path.realpath(__file__))

CONFIG_HEADER = script_dir + '/../src/config.h'
CLOCK_CONFIG_HEADER = script_dir + '/../ext/libmsp/src/include/libmsp/clock-config.h'
HOST_COMM_HEADER = script_dir + '/../src/host_comm.h'
TARGET_COMM_HEADER = script_dir + '/../ext/libedb/src/include/libedb/target_comm.h'

host_comm_header = Header(HOST_COMM_HEADER,
    enums=[
        'USB_CMD',
        'USB_RSP',
        'RETURN_CODE',
        'BREAKPOINT_TYPE',
        'INTERRUPT_SOURCE',
        'ADC_CHAN_INDEX',
        'ENERGY_BREAKPOINT_IMPL',
        'CMP_REF',
        'STREAM',
        'RF_EVENT',
        'PARAM'
    ],
    numeric_macros=[
        'UART_IDENTIFIER_USB'
    ])

target_comm_header = Header(TARGET_COMM_HEADER,
    enums=['INTERRUPT_TYPE'])

config_header = Header(CONFIG_HEADER,
    string_macros=[
        'CONFIG_TIMELOG_TIMER_SOURCE',
        'CONFIG_ADC_TIMER_SOURCE_ACLK',
        'CONFIG_ADC_TIMER_SOURCE_SMCLK',
        'CONFIG_ADC_TIMER_SOURCE_MCLK',
    ],
    numeric_macros=[
        'CONFIG_USB_UART_BAUDRATE',
        'CONFIG_ADC_TIMER_DIV',
        'CONFIG_TIMELOG_TIMER_DIV',
        'CONFIG_TIMELOG_TIMER_DIV_EX'
    ])

clock_config_header = Header(CLOCK_CONFIG_HEADER,
    numeric_macros=[
        'CONFIG_XT1_FREQ',
        'CONFIG_REFO_FREQ',
        'CONFIG_DCOCLKDIV_FREQ',
        'CONFIG_CLK_DIV_SMCLK',
    ])

#VDD = 3.3
VDD = 2.985

d = {
    "host_comm_header": host_comm_header.as_dict(),
    "target_comm_header": target_comm_header.as_dict(),
    "config_header": config_header.as_dict(),
    "clock_config_header": clock_config_header.as_dict(),

    "VDD": VDD,

    "COMPARATOR_REF_VOLTAGE": {
        "VCC" : VDD,
        "VREF_2_5" : 2.5,
        "VREF_2_0" : 2.0,
        "VREF_1_5" : 1.5,
    }
}

fout = open(args.out, "w")
s = json.dump(d, fout, indent=2)
