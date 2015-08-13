#!/usr/bin/python

import wispmon

mon = wispmon.WispMonitor()

restored_vcap = mon.exit_debug_mode()
print "Exited debug mode: restored Vcap = %.4f" % restored_vcap

mon.destroy()
