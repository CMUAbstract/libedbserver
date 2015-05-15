# author: Graham Harvey
# date: 16 April 2015

# This script uses the WISP monitor to read Vcap, the WISP main capacitor
# voltage, as fast as the monitor can for some time period.  The time values
# are represented in WISP monitor SMCLK cycles in 4 bytes here.  The ADC
# values, from 0 to 4095 (12-bit ADC), are represented in two bytes. A time
# value corresponds to the next ADC value.  The delimiters in the data file
# being produced correspond to the UART message descriptors.  The idea is to
# use MATLAB to plot the data in the data file that this script produces.

import wispmon
import atexit

SAMPLE_TIME                     = 2.0 # seconds

HEX_FILE                        = 'data/v.dat' # data file used for plotting with matlab

mon = wispmon.WispMonitor()     # initialize WISP monitor object
fp = open(HEX_FILE, 'wb')
numSamples = 0 # number of Vcap samples we've taken

def cleanup():
    global curCycles, numSamples
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_END) # send a message to stop logging Vcap
    
    # clean up
    fp.close()
    mon.destroy() # closes serial port
    
    # print results summary
    print "Results Summary:"
    print "\tTime:\t\t\t%f sec" % (curCycles * mon.CLK_PERIOD)
    print "\tVcap data points:\t%d" % numSamples
    
    exit()

def main():
    global curCycles, numSamples
    atexit.register(cleanup) # cleanup() will be called just before exiting
    
    buf = bytearray() # initialize empty buffer for serial data
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_BEGIN) # send a message to start logging Vcap
    print("Logging... Ctrl-C to stop")
    
    startCycles = -1 # used for adjusting our time data to when we first receive something
    curCycles = 0 # debugger SMCLK cycles that have passed  (from serial message)
    
    while(curCycles * mon.CLK_PERIOD < SAMPLE_TIME): # loop until SAMPLE_TIME seconds have passed
        bufLen = mon.serial.inWaiting() # get the number of bytes available
        if(bufLen > 0):
            # there are bytes in the buffer
            newBytes = bytearray(mon.serial.read(bufLen)) # read the bytes
            buf.extend(newBytes) # add the bytes to the end of the buffer that we're processing
            
        # try to build an Rx packet
        if(mon.buildRxPkt(buf) == 0):
            # packet construction succeeded
            
            if(mon.rxPkt.descriptor == wispmon.USB_RSP_TIME):
                # received time data
                # Construct the number of cycles from the data.  The least significant byte is first.
                curCycles = (mon.rxPkt.data[3] << 24) | (mon.rxPkt.data[2] << \
                            16) | (mon.rxPkt.data[1] << 8) | (mon.rxPkt.data[0])
                
                if(startCycles == -1):
                    # received first time data - store it for reference
                    startCycles = curCycles
                
                # may need to deal with 32-bit overflow for time data if SAMPLE_TIME is above 195 seconds
                
                curCycles -= startCycles # adjust to when we started
                
                # Construct the data that we'll store in the data file for matlab
                time = bytearray([wispmon.USB_RSP_TIME,
                                  (curCycles & 0xFF000000) >> 24,
                                  (curCycles & 0x00FF0000) >> 16,
                                  (curCycles & 0x0000FF00) >> 8,
                                   curCycles & 0x000000FF])
                
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_VCAP):
                # received Vcap data
                numSamples += 1
                # least significant byte is first
                Vcap = bytearray([wispmon.USB_RSP_VCAP,
                                  mon.rxPkt.data[1], mon.rxPkt.data[0]])
                
                # write both values now to ensure that we don't get a time
                # with no corresponding Vcap value
                fp.write(time + Vcap)
            mon.rxPkt.processed = True

if __name__ == '__main__':
    main()