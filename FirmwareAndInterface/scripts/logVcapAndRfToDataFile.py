# author: Graham Harvey
# date: 16 April 2015

# This script uses the WISP monitor to read Vcap, the WISP main capacitor
# voltage, and RF data as fast as the monitor can for some time period.
# The time values are represented in WISP monitor clock cycles in 4 bytes here.
# The ADC values, from 0 to 4095 (12-bit ADC), are represented in two bytes.
# A time value corresponds to the next ADC value.  The delimiters correspond
# to the UART message descriptors.  The idea is to use MATLAB to plot the
# data in the data file that this script produces.

import wispmon
import atexit

SAMPLE_TIME                     = 2.0 # s

HEX_FILE                        = 'data/vcapAndRf.dat'

mon = wispmon.WispMonitor()
fp = open(HEX_FILE, 'wb')
VcapSamples = 0
rxSamples = 0
txSamples = 0

def cleanup():
    global curCycles, VcapSamples, rxSamples, txSamples
    
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_TX_END) # stop logging RF Tx data
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_RX_END) # stop logging RF Rx data
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_END) # stop logging Vcap
    
    # clean up
    fp.close()
    mon.serial.close()
    
    print "Results:"
    print "\tTime:\t\t\t%f sec" % (curCycles * mon.CLK_PERIOD)
    print "\tVcap data points:\t%d" % VcapSamples
    print "\tRF RX events:\t\t%d" % rxSamples
    print "\tRF TX events:\t\t%d" % txSamples
    exit()

def main():
    global curCycles, VcapSamples, rxSamples, txSamples
    atexit.register(cleanup)
    
    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_BEGIN) # start logging Vcap
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_RX_BEGIN) # start logging RF Rx data
    mon.sendCmd(wispmon.USB_CMD_LOG_RF_TX_BEGIN) # start logging RF Tx data
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
                fp.write(time)
                
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_VCAP):
                # received Vcap data
                VcapSamples += 1
                Vcap = bytearray([wispmon.USB_RSP_VCAP,
                                  mon.rxPkt.data[1], mon.rxPkt.data[0]])
                fp.write(Vcap)
            
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