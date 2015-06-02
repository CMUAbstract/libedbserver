# WISP Monitor Python Class

import serial
import math
from binascii import hexlify

SERIAL_PORT                         = 'COM17'
BAUD_RATE                           = 921600

UART_USB_IDENTIFIER                 = 0xF0

# Serial transmit message descriptors
USB_CMD_GET_VCAP                     = 0x00
USB_CMD_GET_VBOOST                   = 0x01
USB_CMD_GET_VREG                     = 0x02
USB_CMD_GET_VRECT                    = 0x03
USB_CMD_SET_VCAP                     = 0x04
USB_CMD_SET_VBOOST                   = 0x05
USB_CMD_SET_VREG                     = 0x06
USB_CMD_SET_VRECT                    = 0x07
USB_CMD_RELEASE_POWER                = 0x08
USB_CMD_ENTER_ACTIVE_DEBUG           = 0x09
USB_CMD_EXIT_ACTIVE_DEBUG            = 0x0A
USB_CMD_GET_WISP_PC                  = 0x0B
USB_CMD_EXAMINE_MEMORY               = 0x0C
USB_CMD_LOG_VCAP_BEGIN               = 0x0D
USB_CMD_LOG_VCAP_END                 = 0x0E
USB_CMD_LOG_VBOOST_BEGIN             = 0x0F
USB_CMD_LOG_VBOOST_END               = 0x10
USB_CMD_LOG_VREG_BEGIN               = 0x11
USB_CMD_LOG_VREG_END                 = 0x12
USB_CMD_LOG_VRECT_BEGIN              = 0x13
USB_CMD_LOG_VRECT_END                = 0x14
USB_CMD_LOG_RF_RX_BEGIN              = 0x15
USB_CMD_LOG_RF_RX_END                = 0x16
USB_CMD_LOG_RF_TX_BEGIN              = 0x17
USB_CMD_LOG_RF_TX_END                = 0x18
USB_CMD_SEND_RF_TX_DATA              = 0x19
USB_CMD_LOG_WISP_UART_BEGIN          = 0x1A
USB_CMD_LOG_WISP_UART_END            = 0x1B
USB_CMD_ENABLE_PORT_INT_TAG_PWR      = 0x1C
USB_CMD_DISABLE_PORT_INT_TAG_PWR     = 0x1D
USB_CMD_PWM_ON                       = 0x1E
USB_CMD_PWM_OFF                      = 0x1F
USB_CMD_SET_PWM_FREQUENCY            = 0x20
USB_CMD_SET_PWM_DUTY_CYCLE           = 0x21
USB_CMD_LOG_VINJ_BEGIN               = 0x22
USB_CMD_LOG_VINJ_END                 = 0x23
USB_CMD_PWM_HIGH                     = 0x24
USB_CMD_PWM_LOW                      = 0x25
USB_CMD_MONITOR_MARKER_BEGIN         = 0x26
USB_CMD_MONITOR_MARKER_END           = 0x27
USB_CMD_PULSE_AUX_3                  = 0x28
USB_CMD_CHARGE                       = 0x29
USB_CMD_DISCHARGE                    = 0x30

# Serial receive message descriptors
USB_RSP_VCAP                         = 0x00
USB_RSP_VBOOST                       = 0x01
USB_RSP_VREG                         = 0x02
USB_RSP_VRECT                        = 0x03
USB_RSP_SET_POWER_COMPLETE           = 0x04
USB_RSP_RELEASE_POWER_COMPLETE       = 0x05
USB_RSP_WISP_PC                      = 0x06
USB_RSP_WISP_MEMORY                  = 0x07
USB_RSP_RF_RX                        = 0x08
USB_RSP_RF_TX                        = 0x09
USB_RSP_UART_WISP_TO_MONITOR         = 0x0A
USB_RSP_UART_MONITOR_TO_WISP         = 0x0B
USB_RSP_TAG_PWR                      = 0x0C
USB_RSP_TIME                         = 0x0D
USB_RSP_VINJ                         = 0x0E

# Packet construction states
CONSTRUCT_STATE_IDENTIFIER          = 0x00
CONSTRUCT_STATE_DESCRIPTOR          = 0x01
CONSTRUCT_STATE_DATA_LEN            = 0x02
CONSTRUCT_STATE_DATA                = 0x03

class WispMonitor:
    VDD                                 = 3.35 # V
    CLK_FREQ                            = 669 * 32768 # Hz (average)
    CLK_PERIOD                          = 1.0 / CLK_FREQ # seconds

    def __init__(self):
        self.rxPkt = RxPkt()
        self.serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE,
                                    timeout=1)
        self.serial.close()
        self.serial.open()
        
    def destroy(self):
        self.serial.close()
    
    def buildRxPkt(self, buf):
        if(not self.rxPkt.processed):
            return 1
        
        minBufLen = len(buf)
        while minBufLen > 0:
            if(self.rxPkt.constructState == CONSTRUCT_STATE_IDENTIFIER):
                self.rxPkt.identifier = buf.pop(0) # get the identifier byte
                
                if(self.rxPkt.identifier != UART_USB_IDENTIFIER):
                    # unknown identifier - reset state
                    self.rxPkt.processed = True
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return 1 # packet construction failed
                minBufLen -= 1
                self.rxPkt.constructState = CONSTRUCT_STATE_DESCRIPTOR
            elif(self.rxPkt.constructState == CONSTRUCT_STATE_DESCRIPTOR):
                self.rxPkt.descriptor = buf.pop(0) # get descriptor byte
                minBufLen -= 1
                if(self.rxPkt.descriptor in (USB_RSP_SET_POWER_COMPLETE,
                                             USB_RSP_RELEASE_POWER_COMPLETE,
                                             USB_RSP_TAG_PWR, USB_RSP_RF_TX)):
                    # no additional data is needed
                    self.rxPkt.processed = False
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return 0 # packet construction succeeded
                elif(self.rxPkt.descriptor in (USB_RSP_VCAP, USB_RSP_VBOOST,
                                USB_RSP_VREG, USB_RSP_VRECT,
                                USB_RSP_WISP_PC, USB_RSP_WISP_MEMORY,
                                USB_RSP_RF_RX,
                                USB_RSP_UART_WISP_TO_MONITOR, USB_RSP_UART_MONITOR_TO_WISP,
                                USB_RSP_TAG_PWR, USB_RSP_TIME, USB_RSP_VINJ)):
                    # additional data is needed to complete the packet
                    self.rxPkt.constructState = CONSTRUCT_STATE_DATA_LEN
                    continue
                else:
                    # unknown message descriptor
                    self.rxPkt.processed = True
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return 1 # packet construction failed
            elif(self.rxPkt.constructState == CONSTRUCT_STATE_DATA_LEN):
                self.rxPkt.length = buf.pop(0) # get data length
                minBufLen -= 1
                self.rxPkt.constructState = CONSTRUCT_STATE_DATA
                continue
            elif(self.rxPkt.constructState == CONSTRUCT_STATE_DATA):
                if(minBufLen >= self.rxPkt.length):
                    # copy the data
                    self.rxPkt.data = buf[0:self.rxPkt.length]
                    del buf[:self.rxPkt.length]
                    self.rxPkt.processed = False
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return 0 # packet construction succeeded
                else:
                    return 2 # not enough data
            else:
                # unknown state - reset packet construction state
                self.rxPkt.processed = True
                self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                return 1 # packet construction failed
        
        # ran out of data
        return 2 # packet construction will resume next time this function is called
    
    # Note that when sending data, the byte order is reversed.
    # Example: to send a command to set Vcap to 2.2 V, do the following.
    #               descriptor = USB_CMD_SET_VCAP
    #               adc = 2.2 * 2^12 / VDD = 2.2 * 4096 / 3.35 = 2689.91
    #               data = byteReverse(round(adc)) = byteReverse(2690)
    #                    = byteReverse(0x0A82)
    #                    = bytearray([0x82, 0x0A])
    def sendCmd(self, descriptor, data=[]):
        serialMsg = bytearray([UART_USB_IDENTIFIER, descriptor])
        
        dataLen = len(data)
        
        if(dataLen > 0):
            serialMsg += bytearray([dataLen]) + bytearray(data)
        
        self.serial.write(serialMsg)

    def uint16_to_bytes(self, val):
        return [val & 0xFF, (val>> 8) & 0xFF]

    def voltage_to_adc(self, voltage):
        return int(math.ceil(voltage * 4096 / self.VDD))

    def charge(self, target_voltage):

        target_voltage += 0.030 # calibration (TODO: try to find the underlying reason)

        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD_CHARGE, data=cmd_data)

    def discharge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD_DISCHARGE, data=cmd_data)

class RxPkt():
    def __init__(self):
        self.processed = True
        self.constructState = CONSTRUCT_STATE_IDENTIFIER
        self.identifier = 0
        self.descriptor = 0
        self.length = 0
        self.data = None
    
    def printSelf(self):
        print "RxPkt"
        print "\tprocessed =", self.processed
        print "\tidentifier = 0x%02x" % self.identifier
        print "\tdescriptor = 0x%02x" % self.descriptor
        print "\tlength = %d" % self.length
        print "\tdata = 0x%s" % hexlify(self.data)
        