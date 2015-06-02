import sys
import wispmon
import math

mon = wispmon.WispMonitor()

def main():

    target_voltage = 1.8 # default: minimum Vcc for MCU

    if len(sys.argv) > 0:
        target_voltage = float(sys.argv[1])

    target_value = int(math.ceil(target_voltage * 4096 / VCC_V))

    cmd_data = [target_value & 0xFF, (target_value >> 8) & 0xFF]

    print "Charging to %.4f V (target %d)" % (target_voltage, target_value)

    mon.sendCmd(wispmon.USB_CMD_CHARGE, data=cmd_data)
    mon.destroy()

if __name__ == '__main__':
    main()
