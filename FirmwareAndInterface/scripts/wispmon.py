# WISP Monitor Python Class

import serial
import math
import time
from binascii import hexlify

SERIAL_PORT                         = '/dev/ttyUSB0'
BAUD_RATE                           = 921600 # CONFIG_CLOCK_SOURCE_INTERNAL in config.h
#BAUD_RATE                           = 2000000 # CONFIG_CLOCK_SOURCE_CRYSTAL in config.h
#BAUD_RATE                           = 1000000 # CONFIG_CLOCK_SOURCE_CRYSTAL in config.h
#BAUD_RATE                           = 500000 # CONFIG_CLOCK_SOURCE_CRYSTAL in config.h
#BAUD_RATE                           = 115200 # CONFIG_CLOCK_SOURCE_CRYSTAL in config.h

ACLK_FREQ = 32768 # Hz
#SMCLK_FREQ = 24000000 # Hz
SMCLK_FREQ = 21921792 # Hz

TIME_TIMER_FREQ = ACLK_FREQ
#TIME_TIMER_FREQ = SMCLK_FREQ

UART_USB_IDENTIFIER                 = 0xF0

# Channels for sense command
ADC_CHANNEL_INDEX = {
    "cap" : 0,
    "boost" : 1,
    "reg" : 2,
    "rect" : 3,
}

# Serial transmit message descriptors
USB_CMD_SENSE                        = 0x01
USB_CMD_STREAM_BEGIN                 = 0x02
USB_CMD_STREAM_END                   = 0x03
USB_CMD_SET_VCAP                     = 0x04
USB_CMD_SET_VBOOST                   = 0x05
USB_CMD_SET_VREG                     = 0x06
USB_CMD_SET_VRECT                    = 0x07
USB_CMD_RELEASE_POWER                = 0x08
USB_CMD_ENTER_ACTIVE_DEBUG           = 0x09
USB_CMD_EXIT_ACTIVE_DEBUG            = 0x0A
USB_CMD_GET_WISP_PC                  = 0x0B
USB_CMD_EXAMINE_MEMORY               = 0x0C
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
USB_CMD_PWM_HIGH                     = 0x24
USB_CMD_PWM_LOW                      = 0x25
USB_CMD_MONITOR_MARKER_BEGIN         = 0x26
USB_CMD_MONITOR_MARKER_END           = 0x27
USB_CMD_RESET_STATE                  = 0x28
USB_CMD_CHARGE                       = 0x29
USB_CMD_DISCHARGE                    = 0x30
USB_CMD_BREAK_AT_VCAP_LEVEL          = 0x31
USB_CMD_READ_MEM                     = 0x32
USB_CMD_WRITE_MEM                    = 0x33
USB_CMD_CONT_POWER                   = 0x34
USB_CMD_BREAKPOINT                   = 0x35

# Serial receive message descriptors
USB_RSP_VOLTAGE                      = 0x01
USB_RSP_VOLTAGES                     = 0x02
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
USB_RSP_RETURN_CODE                  = 0x0F

# Packet construction states
CONSTRUCT_STATE_IDENTIFIER          = 0x00
CONSTRUCT_STATE_DESCRIPTOR          = 0x01
CONSTRUCT_STATE_DATA_LEN            = 0x02
CONSTRUCT_STATE_DATA                = 0x03

class StreamInterrupted(Exception):
    pass

class WispMonitor:
    VDD                                 = 3.35 # V
    CLK_FREQ                            = TIME_TIMER_FREQ # Hz
    CLK_PERIOD                          = 1.0 / CLK_FREQ # seconds

    def __init__(self):
        self.rxPkt = RxPkt()
        self.serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE,
                                    timeout=1)
        self.serial.close()
        self.serial.open()

        self.rcv_buf = bytearray()
        self.stream_bytes = 0
        
    def destroy(self):
        self.serial.close()
    
    def buildRxPkt(self, buf):
        """Parses packet header and returns whether it is ready or not"""

        minBufLen = len(buf)
        while minBufLen > 0:
            if(self.rxPkt.constructState == CONSTRUCT_STATE_IDENTIFIER):
                self.rxPkt.identifier = buf.pop(0) # get the identifier byte
                
                if(self.rxPkt.identifier != UART_USB_IDENTIFIER):
                    # unknown identifier - reset state
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    raise Exception("packet construction failed: unknown identifier: " +
                            str(self.rxPkt.identifier) + " (exp " + str(UART_USB_IDENTIFIER) + ")")
                minBufLen -= 1
                self.rxPkt.constructState = CONSTRUCT_STATE_DESCRIPTOR
            elif(self.rxPkt.constructState == CONSTRUCT_STATE_DESCRIPTOR):
                self.rxPkt.descriptor = buf.pop(0) # get descriptor byte
                minBufLen -= 1
                if(self.rxPkt.descriptor in (USB_RSP_SET_POWER_COMPLETE,
                                             USB_RSP_RELEASE_POWER_COMPLETE,
                                             USB_RSP_TAG_PWR, USB_RSP_RF_TX)):
                    # no additional data is needed
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return True # packet construction succeeded
                elif(self.rxPkt.descriptor in (USB_RSP_RETURN_CODE,
                                USB_RSP_VOLTAGE, USB_RSP_VOLTAGES,
                                USB_RSP_WISP_PC, USB_RSP_WISP_MEMORY,
                                USB_RSP_RF_RX,
                                USB_RSP_UART_WISP_TO_MONITOR, USB_RSP_UART_MONITOR_TO_WISP,
                                USB_RSP_TAG_PWR, USB_RSP_TIME, USB_RSP_VINJ)):
                    # additional data is needed to complete the packet
                    self.rxPkt.constructState = CONSTRUCT_STATE_DATA_LEN
                    continue
                else:
                    # unknown message descriptor
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    raise Exception("packet construction failed: unknown msg descriptor: " +
                            str(self.rxPkt.descriptor))
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
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    return True # packet construction succeeded
                else:
                    return False # not enough data
            else:
                # unknown state - reset packet construction state
                self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                raise Exception("packet construction failed")
        
        # ran out of data
        return False # packet construction will resume next time this function is called
    
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

    def receive(self):
        if self.buildRxPkt(self.rcv_buf):
            pkt = { "descriptor" : self.rxPkt.descriptor }

            if self.rxPkt.descriptor == USB_RSP_RETURN_CODE:
                pkt["code"] = self.rxPkt.data[0]

            elif self.rxPkt.descriptor == USB_RSP_TIME:
                cycles = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                         (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0])
                pkt["time_sec"] = float(cycles) * self.CLK_PERIOD

            elif self.rxPkt.descriptor == USB_RSP_VOLTAGE:
                adc_reading = (self.rxPkt.data[1] << 8) | self.rxPkt.data[0]
                pkt["voltage"] = self.adc_to_voltage(adc_reading)

            elif self.rxPkt.descriptor == USB_RSP_VOLTAGES:
                num_channels = self.rxPkt.data[0]
                pkt["voltages"] = []
                for i in range(num_channels):
                    adc_reading = (self.rxPkt.data[2 * i + 1] << 8) | self.rxPkt.data[2 * i]
                    pkt["voltages"].append(self.adc_to_voltage(adc_reading))

            elif self.rxPkt.descriptor == USB_RSP_WISP_MEMORY:
                pkt["address"] = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                                 (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0] <<  0)
                pkt["value"] = self.rxPkt.data[4]

            return pkt

        newData = self.serial.read()

        if(len(newData) > 0):
            newBytes = bytearray(newData)
            self.rcv_buf.extend(newBytes)
            self.stream_bytes += len(newData)
        return None

    def receive_reply(self, descriptor):
        reply = None
        while reply is None:
            reply = self.receive()
        if reply["descriptor"] != descriptor:
            raise Exception("unexpected reply: " + \
                    str(reply["descriptor"]) + "(exp " + str(descriptor) + ")")
        if descriptor == USB_CMD_RETURN_CODE: # this one is generic, so handle it here
            if reply["code"] != 0:
                raise Exception("Command failed: return code " + str(reply["code"]))
        return reply

    def flush(self):
        while len(self.serial.read()) > 0:
            pass
        self.rcv_buf = []

    def uint16_to_bytes(self, val):
        return [val & 0xFF, (val>> 8) & 0xFF]

    def bytes_to_uint16(self, bytes):
        return (bytes[1] << 8) | bytes[0]

    def uint32_to_bytes(self, val):
        return [val & 0xFF, (val>> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF]

    def bytes_to_uint32(self, bytes):
        return (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | (bytes[0] << 0)

    def voltage_to_adc(self, voltage):
        return int(math.ceil(voltage * 4096 / self.VDD))

    def adc_to_voltage(self, value):
        return float(value) / 4096 * self.VDD

    def sense(self, channel):
        self.sendCmd(USB_CMD_SENSE, data=[channel])
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def build_stream_cmd_data(self, channels):
        cmd_data = [len(channels)]
        for chan in channels:
            cmd_data.append(chan)
        return cmd_data

    def stream_begin(self, channels):
        self.stream_bytes = 0
        self.stream_start = time.time()
        self.sendCmd(USB_CMD_STREAM_BEGIN, data=self.build_stream_cmd_data(channels))

    def stream_end(self, channels):
        self.sendCmd(USB_CMD_STREAM_END, data=self.build_stream_cmd_data(channels))
        self.flush()

    def stream_datarate_kbps(self):
        return float(self.stream_bytes) / 1000 / (time.time() - self.stream_start)

    def charge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD_CHARGE, data=cmd_data)
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def discharge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD_DISCHARGE, data=cmd_data)
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def enter_debug_mode(self):
        self.sendCmd(USB_CMD_ENTER_ACTIVE_DEBUG)
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def exit_debug_mode(self):
        self.sendCmd(USB_CMD_EXIT_ACTIVE_DEBUG)
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def break_at_vcap_level(self, level):
        level_adc = self.voltage_to_adc(level)
        cmd_data = self.uint16_to_bytes(level_adc)
        self.sendCmd(USB_CMD_BREAK_AT_VCAP_LEVEL, data=cmd_data)
        reply = self.receive_reply(USB_RSP_VOLTAGE)
        return reply["voltage"]

    def breakpoint(self, idx, enable):
        self.sendCmd(USB_CMD_BREAKPOINT, data=[idx, enable])
        self.receive_reply(USB_RSP_RETURN_CODE)

    def read_mem(self, addr):
        cmd_data = self.uint32_to_bytes(addr)
        self.sendCmd(USB_CMD_READ_MEM, data=cmd_data)
        reply = self.receive_reply(USB_RSP_WISP_MEMORY)
        return reply["address"], reply["value"]

    def write_mem(self, addr, value):
        cmd_data = self.uint32_to_bytes(addr) + [value]
        self.sendCmd(USB_CMD_WRITE_MEM, data=cmd_data)
        reply = self.receive_reply(USB_RSP_WISP_MEMORY)
        return reply["address"], reply["value"]

    def cont_power(self, on):
        cmd_data = [on]
        self.sendCmd(USB_CMD_CONT_POWER, data=cmd_data)
        self.receive_reply(USB_RSP_RETURN_CODE)


class RxPkt():
    def __init__(self):
        self.constructState = CONSTRUCT_STATE_IDENTIFIER
        self.identifier = 0
        self.descriptor = 0
        self.length = 0
        self.data = None
    
    def printSelf(self):
        print "RxPkt"
        print "\tidentifier = 0x%02x" % self.identifier
        print "\tdescriptor = 0x%02x" % self.descriptor
        print "\tlength = %d" % self.length
        print "\tdata = 0x%s" % hexlify(self.data)

