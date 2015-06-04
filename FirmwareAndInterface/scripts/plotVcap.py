# This script uses the WISP monitor to read Vcap, the WISP main capacitor
# voltage, as fast as the monitor can for some time period.  The values,
# from 0 to 4095 (12-bit ADC), are then plotted using the Python library
# matplotlib.  Time is also obtained from the WISP monitor.

import wispmon
import atexit
import matplotlib.pyplot as plt

SAMPLE_TIME                     = 1.0 # s

mon = wispmon.WispMonitor()
t = []
Vcap = []

def cleanup():
    global Vcap, t, curTime
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_END) # stop logging Vcap
    
    # clean up
    mon.destroy()
    
    print "Results:"
    print "\tTime:\t\t\t%f sec" % curTime
    print "\tVcap data points:\t%d" % len(Vcap)
    
    # make the lists the same length
    lenT = len(t)
    lenVcap = len(Vcap)
    if lenT > lenVcap:
        t = t[:lenVcap]
    elif lenT < lenVcap:
        Vcap = Vcap[:lenT]
    
    # plot the data
    plt.plot(t, Vcap)
    plt.ylabel('Vcap (V)')
    plt.show()
    
    exit()

def main():
    global Vcap, t, curTime
    
    atexit.register(cleanup)
    
    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_LOG_VCAP_BEGIN) # start logging Vcap
    print("Logging... Ctrl-C to stop")
    
    startCycles = -1
    curCycles = 0
    curTime = 0
    
    while(curTime < SAMPLE_TIME):
        bufLen = mon.serial.inWaiting() # get the number of bytes available
        if(bufLen > 0):
            newBytes = bytearray(mon.serial.read(bufLen))
            buf.extend(newBytes)
        
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
                
                curTime = curCycles * mon.CLK_PERIOD
                t += [curTime]
                
            elif(mon.rxPkt.descriptor == wispmon.USB_RSP_VCAP):
                # received Vcap data
                Vcap_adc = (mon.rxPkt.data[1] << 8) | mon.rxPkt.data[0]
                Vcap += [float(Vcap_adc) / 4096 * mon.VDD]
            mon.rxPkt.processed = True

if __name__ == '__main__':
    main()
