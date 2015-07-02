#!/usr/bin/python

import sys
import wispmon

mon = wispmon.WispMonitor()

vcap = mon.get_vcap()
print "Vcap = %.4f V" % (vcap)

mon.destroy()
