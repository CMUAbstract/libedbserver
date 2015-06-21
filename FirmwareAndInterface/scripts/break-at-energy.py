import sys
import wispmon
import time

mon = wispmon.WispMonitor()

def main():

    target_voltage = float(sys.argv[1])
    vcap_level = float(sys.argv[2])

    # Pre-charge because to the given voltage
    print "Charging to %.4f V..." % (target_voltage)
    actual_voltage = mon.charge(target_voltage)
    print "Charged to %.4f V" % (actual_voltage)

    # WISP Boot latency (TODO: why is it taking this long? about 6 ms)
    time.sleep(0.015) # let it run

    actual_vcap = mon.break_at_vcap_level(vcap_level)
    print "Reached Vcap = %.4f, entered debug mode" % actual_vcap
    time.sleep(0.020)
    restored_vcap = mon.exit_debug_mode()
    print "Exited debug mode: restored Vcap = %.4f V" % restored_vcap

    mon.destroy()

if __name__ == '__main__':
    main()
