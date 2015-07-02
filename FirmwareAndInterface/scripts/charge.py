import sys
import wispmon

mon = wispmon.WispMonitor()

def main():

    target_voltage = 1.8 # default: minimum Vcc for MCU

    if len(sys.argv) > 0:
        target_voltage = float(sys.argv[1])

    print "Charging to %.4f V..." % (target_voltage)
    actual_voltage = mon.charge(target_voltage)
    print "Charged to %.4f V" % (actual_voltage)

    mon.destroy()

if __name__ == '__main__':
    main()
