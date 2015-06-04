import sys
import wispmon
import time

mon = wispmon.WispMonitor()

def main():

    target_voltage_enter = 2.0
    target_voltage_active = 2.4
    target_voltage_exit = 2.0

    if len(sys.argv) > 1:
        target_voltage_enter = float(sys.argv[1])
        target_voltage_active = float(sys.argv[2])
        target_voltage_exit = float(sys.argv[3])

    print "Charging to %.4f V" % (target_voltage_enter)
    mon.charge(target_voltage_enter)

    time.sleep(0.030)

    print "Charging to %.4f V" % (target_voltage_active)
    mon.charge(target_voltage_active)

    time.sleep(0.030)

    print "Discharging to %.4f V" % (target_voltage_exit)
    mon.discharge(target_voltage_exit)

    mon.destroy()

if __name__ == '__main__':
    main()
