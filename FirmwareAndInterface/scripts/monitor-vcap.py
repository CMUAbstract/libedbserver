# author: Graham Harvey
# date: 16 April 2015

# This script uses the WISP monitor to read Vcap, the WISP main capacitor
# voltage, as fast as the monitor can for some time period.  The values,
# from 0 to 4095 (12-bit ADC), are written to a text file in a human-readable
# format.  Each line contains a new sample.
# Time is also obtained from the WISP monitor.

import wispmon
import signal
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--duration', '-d', type=float, default=-1,
    help='how long to record data (seconds), by default indefinitely')
parser.add_argument('--out', '-o',
    help='output filename where to save data, standard out if not specified')
args = parser.parse_args()

mon = wispmon.WispMonitor()

if args.out is not None:
    fp = open(args.out, 'w')
else:
    fp = sys.stdout

numSamples = 0
total_bytes = 0

def cleanup(interrupted):
    global curTime, numSamples, total_bytes
    
    if interrupted:
        print " Interrupted"
    else:
        print "Stopping"

    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_END) # stop logging Vcap
    
    # clean up
    mon.destroy()

    if curTime > 0:
        data_rate_kbps = float(total_bytes) / 1000 / curTime
    else:
        data_rate_kbps = 0
    print "%d data points (%d B) in %f seconds: %.02f KB/s" % \
            (numSamples, total_bytes, curTime, data_rate_kbps)

def sigint_handler(signum, frame):
    cleanup(interrupted=True)
    exit()

def main():
    global curTime, numSamples, total_bytes

    signal.signal(signal.SIGINT, sigint_handler)

    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_BEGIN) # start logging Vcap
    print("Logging... Ctrl-C to stop")
    startCycles = -1
    curCycles = 0
    curTime = 0

    fp.write("time,Vcap\n")
    
    while(args.duration < 0 or curTime < args.duration):
        bufLen = mon.serial.inWaiting() # get the number of bytes available
        if(bufLen == 0):
            continue

        newBytes = bytearray(mon.serial.read(bufLen))
        buf.extend(newBytes)

        total_bytes += len(newBytes)
        if args.out is not None: # only show progress if not outputing to console
            print "\r", total_bytes, "B",

        while mon.buildRxPkt(buf):
            # packet construction succeeded
            
            if(mon.rxPkt.descriptor == wispmon.USB_RSP_TIME):
                # received time data
                curCycles = (mon.rxPkt.data[3] << 24) | (mon.rxPkt.data[2] << 16) | \
                            (mon.rxPkt.data[1] << 8) | (mon.rxPkt.data[0])
                
                if(startCycles == -1):
                    # first time data - store it for reference
                    startCycles = curCycles
                
                # may need to deal with 32-bit overflow for time data
                
                curCycles -= startCycles # adjust to when we started
                curTime = curCycles * mon.CLK_PERIOD
            
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_VCAP):
                # received Vcap data
                numSamples += 1
                Vcap_adc = (mon.rxPkt.data[1] << 8) | mon.rxPkt.data[0]
                Vcap = float(Vcap_adc) / 4096 * mon.VDD
                
                # write both strings now to ensure that we don't get a time
                # with no corresponding Vcap value
                fp.write("%f,%f\n" % (curTime, Vcap))
            mon.rxPkt.processed = True

    cleanup(interrupted=False)

if __name__ == '__main__':
    main()
