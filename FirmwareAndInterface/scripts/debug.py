#!/usr/bin/python

import sys
import traceback
import wispmon
import os
import atexit
import readline # loading this causes raw_input to offer a rich prompt

monitor = None
active_mode = False

def to_int(s):
    if s.startswith("0x"):
        return int(s, 16)
    else:
        return int(s)

def match_keyword(part, words):
    match = None
    for word in words:
        if word.startswith(part):
            if match is not None:
                raise Exception("Ambiguous keyword: " + part)
            match = word
    return match

def print_interrupt_context(context):
    print "Interrupted:", context.type, "id:", context.id,
    if context.saved_vcap is not None:
        print "Vcap_saved = %.4f" % context.saved_vcap
    else:
        print

def cmd_echo(mon, args):
    print args

def cmd_attach(mon):
    global monitor
    monitor = wispmon.WispMonitor()

def cmd_detach(mon):
    mon.destroy()

def cmd_power(mon, state):
    mon.cont_power(state == "on")

def cmd_sense(mon, channel):
    print mon.sense(wispmon.ADC_CHAN_INDEX[channel.upper()])

def cmd_stream(mon, out_file, duration_sec, *streams):
    if duration_sec == "-":
        duration_sec = None # stream indefinitely
    else:
        duration_sec = float(duration_sec)

    streams = map(str.upper, streams)

    if out_file == "-":
        fp = sys.stdout
        silent = True
    else:
        fp = open(out_file, "w")
        #silent = False
        silent = True # TODO: temprorary

    try:
        mon.stream(streams, duration_sec=duration_sec, out_file=fp, silent=silent)
    except KeyboardInterrupt:
        pass # this is a clean termination

def cmd_charge(mon, target_voltage, method="adc"):
    target_voltage = float(target_voltage)
    if method == "adc":
        vcap = mon.charge(target_voltage)
        print "Vcap = %.4f" % vcap
    elif method == "cmp":
        mon.charge_cmp(target_voltage)
    else:
        raise Exception("Invalid charger method: " + method)

def cmd_discharge(mon, target_voltage, method="adc"):
    target_voltage = float(target_voltage)
    if method == "adc":
        vcap = mon.discharge(target_voltage)
        print "Vcap = %.4f" % vcap
    elif method == "cmp":
        mon.discharge_cmp(target_voltage)
    else:
        raise Exception("Invalid charger method: " + method)

def cmd_int(mon):
    global active_mode
    try:
        saved_vcap = mon.interrupt()
        print "Vcap_saved = %.4f" % saved_vcap
        active_mode = True
    except KeyboardInterrupt:
        pass

def cmd_cont(mon):
    global active_mode
    restored_vcap = mon.exit_debug_mode()
    print "Vcap_restored = %.4f" % restored_vcap
    active_mode = False

def cmd_ebreak(mon, target_voltage, impl="adc"):
    global active_mode
    target_voltage = float(target_voltage)
    saved_vcap = mon.break_at_vcap_level(target_voltage, impl.upper())
    print "Vcap_saved = %.4f" % saved_vcap
    active_mode = True

def cmd_break(mon, type, idx, op, energy_level=None):
    idx = int(idx)
    enable = "enable".startswith(op)
    type = match_keyword(type.upper(), wispmon.BREAKPOINT_TYPE.keys())
    energy_level = float(energy_level) if energy_level is not None else None
    mon.breakpoint(type, idx, enable, energy_level)

def cmd_wait(mon):
    """Wait to enter active debug mode"""
    global active_mode
    try:
        int_context = mon.wait_for_interrupt()
        print_interrupt_context(int_context)
        active_mode = True
    except KeyboardInterrupt:
        pass

def cmd_intctx(mon, source="debugger"):
    source = match_keyword(source.upper(), wispmon.INTERRUPT_SOURCE)
    int_context = mon.get_interrupt_context(source)
    print_interrupt_context(int_context)

def cmd_read(mon, addr, len):
    addr = int(addr, 16)
    len = int(len)
    addr, value = mon.read_mem(addr, len)
    print "0x%08x:" % addr,
    for byte in value:
        print "0x%02x" % byte,
    print

def cmd_write(mon, addr, *value):
    addr = int(addr, 16)
    value = map(to_int, value)
    mon.write_mem(addr, value)

def cmd_pc(mon):
    print "0x%08x" % mon.get_pc()

def cmd_secho(mon, value):
    value = int(value, 16)
    print mon.serial_echo(value)

def compose_prompt(active_mode):
    if active_mode:
        return "*> "
    return "> "

cmd_hist_file = os.path.join(os.path.expanduser("~"), ".wispmon_history")
try:
    readline.read_history_file(cmd_hist_file)
except IOError:
    pass
atexit.register(readline.write_history_file, cmd_hist_file)

while True:
    try:
        line = raw_input(compose_prompt(active_mode))
    except EOFError:
        print # print a newline to be nice to the shell
        break
    except KeyboardInterrupt:
        print # move to next line
        continue

    line = line.strip()
    if len(line) == 0: # new-line character only (blank command)
        continue
    cmd_lines = line.split(';')
    try:
        for cmd_line in cmd_lines:
            tokens = cmd_line.split()
            cmd = tokens[0]
            glob = globals()
            glob["cmd_" + cmd](monitor, *tokens[1:])
    except Exception as e:
        print type(e)
        print traceback.format_exc()
