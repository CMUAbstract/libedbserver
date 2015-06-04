import sys
import wispmon
import time

mon = wispmon.WispMonitor()

def main():

#    target_voltage = 1.8 # default: minimum Vcc for MCU
#
#    if len(sys.argv) > 0:
#        target_voltage = float(sys.argv[1])
#
#    print "Charging to %.4f V" % (target_voltage)
#    mon.charge(target_voltage)

    mon.enter_debug_mode()

    #time.sleep(0.100)

    #mon.exit_debug_mode()

    mon.destroy()

if __name__ == '__main__':
    main()
