author: Graham Harvey
date: 24 April 2015

This directory contains the scripts used to collect and process data from
the WISP debugger.

The structure of the data files that are produced is simple.  Each piece of
data (time, Vcap, etc.) is stored with just a delimiter and the value.  The
delimiter is the same as the corresponding UART message descriptor.  For
example, a time value is stored with USB_RSP_TIME (0x0D), followed by the 32-bit
time value.  Voltages are stored as 16-bit values, times are stored as 32-bit
values, and RF data is stored as 8-bit values.  This simpler file structure will
make it easier for MATLAB to plot the data.  It is less robust than the UART
message structure, but my hope is that the Python scripts will be able to sort
out any errors and prevent corrupt data from being written to the data files.

The following list contains a brief summary of what each file is.

=======================================
Python scripts:
=======================================
logRfToDataFile.py - Collects only RF (Rx and Tx) data from the debugger for a specified amount of time and stores it in a data file

logVboostToDataFile.py - Collects only Vboost data for a specified amount of time and stores it in a data file

logVcapAndRfToDataFile.py - Collects both RF and Vcap data for a specified amount of time and stores it in a data file

logVcapToDataFile.py - Collects Vcap data for a specified amount of time and stores it in a data file

logVcapToTextFile.py - Collects Vcap data for a specified amount of time and stores it in a text file in a human-readable format

logVcapVrectToDataFile.py - Collects both Vcap and Vrect data for a specified amount of time and stores it in a data file

logVrectToDataFile.py - Collects Vrect data for a specified amount of time and stores it in a data file

logVregToDataFile.py - Collects Vreg data for a specified amount of time and stores it in a data file

plotVcap.py - Collects Vcap data for a specified amount of time and plots it with the Python matplotlib library

pwmVcapMap.py - Contains a map from PWM frequencies and PWM duty cycles to the resulting Vcap level once the WISP capacitor is charged, measured with an oscilloscope

wispmon.py - Contains the WISP monitor Python class to provide an abstraction layer and programming API

=======================================
MATLAB scripts:
=======================================
monitor.m - Contains constants used to plot data

plotRfFromDataFile.m - Plots RF data from a data file that logRfToDataFile.py produces

plotVcapAndRfFromDataFile.m - Plots Vcap and RF data from a data file that logVcapAndRfToDataFile.py produces

plotVFromDataFile.m - Plots any of Vcap, Vboost, Vrect, Vreg, and/or Vinj on the same axes
