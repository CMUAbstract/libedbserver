import sys
import wispmon
import time

mon = wispmon.WispMonitor()

def main():

    # TODO: think about this: at any given time, the WISP is most probably off,
    # so it is a bit weird to have a use case that just says "enter debug mode now"
    # precharging seems necessary in this case. Energy-break points make sense here.

    # Pre-charge because to the given voltage if requested
    if len(sys.argv) > 0:
        target_voltage = float(sys.argv[1])
        print "Charging to %.4f V..." % (target_voltage)
        actual_voltage = mon.charge(target_voltage)
        print "Charged to %.4f V" % (actual_voltage)

        # WISP Boot latency (TODO: why is it taking this long? about 6 ms)
        time.sleep(0.015) # let it run

    saved_vcap = mon.enter_debug_mode()
    print "Entered debug mode: saved Vcap = %.4f V" % saved_vcap

    #time.sleep(0.020)

    time.sleep(0.010)

    # write-read a memory location
    addr = 0x1d0d
    value = 0xab
    addr = mon.write_mem(addr, value)
    print "Mem: write: 0x%x: 0x%x" % (addr, value)
    value = mon.read_mem(addr)
    print "Mem: read: 0x%x: 0x%x" % (addr, value)

    time.sleep(0.010)

    restored_vcap = mon.exit_debug_mode()
    print "Exited debug mode: restored Vcap = %.4f V" % restored_vcap

    time.sleep(0.015)

    saved_vcap = mon.enter_debug_mode()
    print "Entered debug mode: saved Vcap = %.4f V" % saved_vcap
    time.sleep(0.020)
    restored_vcap = mon.exit_debug_mode()
    print "Exited debug mode: restored Vcap = %.4f V" % restored_vcap

    mon.destroy()

if __name__ == '__main__':
    main()
