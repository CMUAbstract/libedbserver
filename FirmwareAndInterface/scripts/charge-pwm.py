import sys
import wispmon
import math

mon = wispmon.WispMonitor()

def main():

    VCC_V = 3.3
    CLK_HZ = 21921792

    R = 390.0
    C = 4.7e-6

    LOW_PASS_FACTOR = 10 # PWM period >> RC for low pass, by this factor
    
    pwm_period = int((R*C / LOW_PASS_FACTOR) * CLK_HZ)

    target_voltage = 1.8 # default: minimum Vcc for MCU

    if len(sys.argv) > 0:
        target_voltage = float(sys.argv[1])
        if len(sys.argv) > 2:
            pwm_freq = int(sys.argv[2])

    target_value = int(math.ceil(target_voltage * 4096 / VCC_V))
    pwm_duty_cycle = int(math.ceil(target_voltage / VCC_V * pwm_period))

    cmd_data = [target_value & 0xFF, (target_value >> 8) & 0xFF,
                pwm_period & 0xFF, (pwm_period >> 8) & 0xFF,
                pwm_duty_cycle & 0xFF, (pwm_duty_cycle  >> 8) & 0xFF]

    print "Charging to %.4f V (target %d, PWM period %d, PWM duty cycle %d)" % \
            (target_voltage, target_value, pwm_period, pwm_duty_cycle)

    mon.sendCmd(wispmon.USB_CMD_CHARGE, data=cmd_data)
    mon.destroy()

if __name__ == '__main__':
    main()
