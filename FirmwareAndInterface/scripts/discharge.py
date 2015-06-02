import sys
import wispmon
import math

mon = wispmon.WispMonitor()

def main():
    
    if len(sys.argv) > 0:
        target_voltage = float(sys.argv[1])
    else:
        target_voltage = 1.8 # default: minimum Vcc for MCU

    target_value = int(math.ceil(target_voltage * 4096 / 3.35))
    cmd_data = [target_value & 0xFF, (target_value >> 8) & 0xFF]

    print "Discharging to %.4f V (%d)" % (target_voltage, target_value)

    mon.sendCmd(wispmon.USB_CMD_DISCHARGE, data=cmd_data)
    mon.destroy()

if __name__ == '__main__':
    main()
