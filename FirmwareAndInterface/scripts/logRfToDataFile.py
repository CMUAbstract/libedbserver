# author: Graham Harvey
# date: 16 April 2015

# This script uses the WISP monitor to get RF data for some time period.
# The time values are represented in WISP monitor clock cycles in 4 bytes here.
# The delimiters correspond to the UART message descriptors.  The idea is to
# use MATLAB to plot the data in the data file that this script produces.

import wispmon
import atexit

# sample time max is 195.922272 sec unless you modify this code to handle overflows
SAMPLE_TIME                     = 2.0

HEX_FILE                        = 'data/rf.dat'

mon = wispmon.WispMonitor()
fp = open(HEX_FILE, 'wb')
rxSamples = 0
txSamples = 0

def cleanup():
    global curCycles, rxSamples, txSamples
    
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_TX_END) # stop logging RF Tx data
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_RX_END) # stop logging RF Rx data
    
    # clean up
    fp.close()
    mon.destroy()
    
    print "Results:"
    print "\tTime:\t\t\t%f sec" % (curCycles * mon.CLK_PERIOD)
    print "\tRF RX events:\t\t%d" % rxSamples
    print "\tRF TX events:\t\t%d" % txSamples
    
    exit()

def main():
    global curCycles, rxSamples, txSamples
    atexit.register(cleanup)
    
    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_RX_BEGIN) # start logging RF Rx data
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_TX_BEGIN) # start logging RF Tx data
    print("Logging... Ctrl-C to stop")
    
    startCycles = -1
    curCycles = 0

    while(curCycles * mon.CLK_PERIOD < SAMPLE_TIME):
        bufLen = mon.serial.inWaiting() # get the number of bytes available
        if(bufLen > 0):
            # add the new bytes to the software buffer
            newBytes = bytearray(mon.serial.read(bufLen))
            buf.extend(newBytes)
            
        # try to build an Rx packet
        if mon.buildRxPkt(buf):
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
                fp.write(time)
                
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_RF_RX):
                # received RF Rx data
                rxSamples += 1
                rx = bytearray([wispmon.USB_RSP_RF_RX, mon.rxPkt.data[0]])
                fp.write(rx);
            
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_RF_TX):
                # received RF Tx data
                txSamples += 1
                tx = bytearray([wispmon.USB_RSP_RF_TX])
                fp.write(tx)
                
            mon.rxPkt.processed = True

if __name__ == '__main__':
    main()