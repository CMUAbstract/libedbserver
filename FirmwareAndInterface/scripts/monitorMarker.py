# author: Graham Harvey
# date: 16 April 2015

# This script instructs the WISP monitor to monitor a code marker pin
# for a set amount of time. No data is excahnged.

import wispmon
import atexit


SAMPLE_TIME                 = 80.0 # s

mon = wispmon.WispMonitor()

numSamples = 0

def cleanup():
    global numSamples
    
    mon.sendCmd(wispmon.USB_CMD_MONITOR_MARKER_END)
    
    # clean up
    mon.destroy()
    
    exit()

def main():
    global numSamples
    atexit.register(cleanup)
    buf = bytearray()
    
    mon.sendCmd(wispmon.USB_CMD_MONITOR_MARKER_BEGIN)
    print("Monitoring... Ctrl-C to stop")

    while True:
        pass

if __name__ == '__main__':
    main()
