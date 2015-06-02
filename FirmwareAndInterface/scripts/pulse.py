import wispmon

mon = wispmon.WispMonitor()

def main():
    
    mon.sendCmd(wispmon.USB_CMD_PULSE_AUX_3)
    mon.destroy()

if __name__ == '__main__':
    main()
