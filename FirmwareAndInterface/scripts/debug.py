#!/usr/bin/python

import sys
import traceback
import wispmon

monitor = None
active_mode = False

def to_int(s):
    if s.startswith("0x"):
        return int(s, 16)
    else:
        return int(s)

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
    print mon.sense(wispmon.ADC_CHANNEL_INDEX[channel])

def cmd_stream(mon, out_file, duration_sec, *channels):
    if duration_sec == "-":
        duration_sec = -1 # stream indefinitely
    else:
        duration_sec = float(duration_sec)
    print "chans=", channels
    channel_indexes = map(lambda c: wispmon.ADC_CHANNEL_INDEX[c], channels)
    print "chansidx=", channel_indexes
    if out_file == "-":
        fp = sys.stdout
    else:
        fp = open(out_file)

    streaming = True
    mon.stream_begin(channel_indexes)
    print("Logging... Ctrl-C to stop")

    start_time_sec = None
    time_sec = 0
    num_samples = 0
    total_bytes = 0

    fp.write("time_sec," + ",".join(channels) + "\n")
    
    while streaming and (duration_sec < 0 or time_sec < duration_sec):
        while streaming:
            try:
                pkt = mon.receive()
            except KeyboardInterrupt:
                streaming = False
                break

            if pkt is None:
                continue
            
            if(pkt["descriptor"] == wispmon.USB_RSP_TIME):
                if start_time_sec is None: # first time data - store it for reference
                    start_time_sec = pkt["time_sec"]
                time_sec = pkt["time_sec"] - start_time_sec # adjust to when we started
            
            elif(pkt["descriptor"] == wispmon.USB_RSP_VOLTAGES):
                num_samples += 1
                fp.write("%f,%s\n" % (time_sec, ",".join(map(lambda v: "%.4f" % v, pkt["voltages"]))))

            if fp != sys.stdout:
                print "\r%.2f KB/s" % mon.stream_datarate_kbps()

        mon.stream_end(channel_indexes)
        print "%d samples in %f seconds (target time)" % (num_samples, time_sec)

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

def cmd_ebreak(mon, target_voltage):
    global active_mode
    target_voltage = float(target_voltage)
    saved_vcap = mon.break_at_vcap_level(target_voltage)
    print "Vcap_saved = %.4f" % saved_vcap
    active_mode = True

def cmd_break(mon, idx, op):
    idx = int(idx)
    enable = "enable".startswith(op)
    mon.breakpoint(idx, enable)

def cmd_wait(mon):
    """Wait to enter active debug mode"""
    global active_mode
    try:
        pkt = mon.receive_reply(wispmon.USB_RSP_INTERRUPTED)
        print "Vcap_saved = %.4f" % pkt["saved_vcap"]
        active_mode = True
    except KeyboardInterrupt:
        pass

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

def print_prompt(active_mode=False):
    if active_mode:
        print "*> ",
    else:
        print "> ",

while True:
    print_prompt(active_mode)
    line = sys.stdin.readline()
    if len(line) == 0: # EOF
        break
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
