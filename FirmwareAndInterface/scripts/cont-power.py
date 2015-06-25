#!/usr/bin/python

import sys
import wispmon

mon = wispmon.WispMonitor()

power_on = int(sys.argv[1]) # 1 - on, 0 - off
vcap = mon.cont_power(power_on)
print "Continuous power %s: Vcap = %.4f V" % ("on" if power_on else "off", vcap)

mon.destroy()
