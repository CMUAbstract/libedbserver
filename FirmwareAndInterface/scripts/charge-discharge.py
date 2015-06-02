import sys
import wispmon
import math
import time

mon = wispmon.WispMonitor()

VCC_V = 3.3

def make_cmd_data(target_voltage):
    target_value = int(math.ceil(target_voltage * 4096 / VCC_V))
    return [target_value & 0xFF, (target_value >> 8) & 0xFF]

def main():

    target_voltage_enter = 2.0
    target_voltage_active = 2.4
    target_voltage_exit = 2.0

    if len(sys.argv) > 1:
        target_voltage_enter = float(sys.argv[1])
        target_voltage_active = float(sys.argv[2])
        target_voltage_exit = float(sys.argv[3])

    print "Charging to %.4f V" % (target_voltage_enter)
    mon.sendCmd(wispmon.USB_CMD_CHARGE, data=make_cmd_data(target_voltage_enter))

    time.sleep(0.030)

    print "Charging to %.4f V" % (target_voltage_active)
    mon.sendCmd(wispmon.USB_CMD_CHARGE, data=make_cmd_data(target_voltage_active))

    time.sleep(0.030)

    print "Discharging to %.4f V" % (target_voltage_exit)
    mon.sendCmd(wispmon.USB_CMD_DISCHARGE, data=make_cmd_data(target_voltage_exit))

    mon.destroy()

if __name__ == '__main__':
    main()
