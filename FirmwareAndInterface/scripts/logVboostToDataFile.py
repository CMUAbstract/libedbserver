# author: Graham Harvey
# date: 22 April 2015

# This script uses the WISP monitor to read Vboost, the WISP boost
# voltage, as fast as the monitor can for some time period.  The time values
# are represented in WISP monitor clock cycles in 4 bytes here.
# The ADC values, from 0 to 4095 (12-bit ADC), are represented in two bytes.
# A time value corresponds to the next ADC value.  The delimiters correspond
# to the UART message descriptors.  The idea is to use MATLAB to plot the
# data in the data file that this script produces.

import wispmon
import atexit

SAMPLE_TIME                     = 2.0 # s

HEX_FILE                        = 'data/vboost.dat'

mon = wispmon.WispMonitor()
fp = open(HEX_FILE, 'wb')
numSamples = 0

def cleanup():
    global curCycles, numSamples
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VBOOST_END) # stop logging Vboost
    
    # clean up
    fp.close()
    mon.destroy()
    
    print "Results:"
    print "\tTime:\t\t\t%f sec" % (curCycles * mon.CLK_PERIOD)
    print "\tVboost data points:\t%d" % numSamples
    
    exit()

def main():
    global curCycles, numSamples
    atexit.register(cleanup)
    
    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VBOOST_BEGIN) # start logging Vboost
    print("Logging... Ctrl-C to stop")
    
    startCycles = -1
    curCycles = 0
    
    while(curCycles * mon.CLK_PERIOD < SAMPLE_TIME):
        bufLen = mon.serial.inWaiting() # get the number of bytes available
        if(bufLen > 0):
            newBytes = bytearray(mon.serial.read(bufLen))
            buf.extend(newBytes)
            
        # try to build an Rx packet
        if(mon.buildRxPkt(buf) == 0):
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
                               
                time = bytearray([wispmon.USB_RSP_TIME,
                                  (curCycles & 0xFF000000) >> 24,
                                  (curCycles & 0x00FF0000) >> 16,
                                  (curCycles & 0x0000FF00) >> 8,
                                   curCycles & 0x000000FF])
                
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_VBOOST):
                # received Vboost data
                numSamples += 1
                Vboost = bytearray([wispmon.USB_RSP_VBOOST,
                                  mon.rxPkt.data[1], mon.rxPkt.data[0]])
                
                # write both values now to ensure that we don't get a time
                # with no corresponding Vboost value
                fp.write(time + Vboost)
            mon.rxPkt.processed = True

if __name__ == '__main__':
    main()