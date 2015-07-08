# WISP Monitor Python Class

import serial
import math
import time
import re
from binascii import hexlify

CONFIG_HEADER = '../firmware/src/config.h'
HOST_COMM_HEADER = '../firmware/src/host_comm.h'
TARGET_COMM_HEADER = '../libdebug/src/include/libdebug/target_comm.h'

SERIAL_PORT                         = '/dev/ttyUSB0'

VDD = 3.3

COMPARATOR_REF_VOLTAGE = {
    "VCC" : VDD,
    "VREF_2_5" : 2.5,
    "VREF_2_0" : 2.0,
    "VREF_1_5" : 1.5,
}

# Packet construction states
CONSTRUCT_STATE_IDENTIFIER          = 0x00
CONSTRUCT_STATE_DESCRIPTOR          = 0x01
CONSTRUCT_STATE_DATA_LEN            = 0x02
CONSTRUCT_STATE_DATA                = 0x03

def key_lookup(d, value):
    for k in d:
        if d[k] == value:
            return k
    return None

def number_from_string(s):
    if s.startswith('0x'):
        val = int(s, 16)
    else:
        val = int(s)
    return val

def parse_enum(header_file, prefix):
    enum_dict = dict()
    for line in open(header_file):
        m = re.match(r'^\s*' + prefix + r'_(?P<name>\w+)\s*=\s*(?P<value>[0-9xA-F]+)', line)
        if m is None:
            continue
        name = m.group('name')
        enum_dict[name] = number_from_string(m.group('value'))
    return enum_dict

def parse_def(header_file, name, cast_func=number_from_string):
    for line in open(header_file):
        m = re.match(r'^\s*#define\s+' + name + '\s+(?P<value>\w+)\s*', line)
        if m is None:
            continue
        return cast_func(m.group('value'))
    raise Exception("Macro '" + name + "' not found in '" + header_file + "'")

def parse_config_def(name):
    return parse_def(CONFIG_HEADER, name)
def parse_config_def_str(name):
    return parse_def(CONFIG_HEADER, name, cast_func=str)

def parse_host_comm_enum(prefix):
    return parse_enum(HOST_COMM_HEADER, prefix)
def parse_host_comm_def(name):
    return parse_def(HOST_COMM_HEADER, name)

def parse_target_comm_enum(prefix):
    return parse_enum(TARGET_COMM_HEADER, prefix)
def parse_target_comm_def(name):
    return parse_def(TARGET_COMM_HEADER, name)

CONFIG_USB_UART_BAUDRATE = parse_config_def('CONFIG_USB_UART_BAUDRATE')
CONFIG_TIMELOG_TIMER_SOURCE = parse_config_def_str('CONFIG_TIMELOG_TIMER_SOURCE')

# Cutting corners a bit: assume that ACLK is sourced either from XT1 or REFO
if CONFIG_TIMELOG_TIMER_SOURCE == 'TASSEL__ACLK':
    assert parse_config_def('CONFIG_XT1_FREQ') == parse_config_def('CONFIG_REFO_FREQ')
    TIME_TIMER_FREQ = parse_config_def('CONFIG_XT1_FREQ')
elif CONFIG_TIMELOG_TIMER_SOURCE == 'TASSEL__SMCLK':
    TIME_TIMER_FREQ = parse_config_def('CONFIG_DCOCLKDIV_FREQ')

USB_CMD = parse_host_comm_enum('USB_CMD')
USB_RSP = parse_host_comm_enum('USB_RSP')
RETURN_CODE = parse_host_comm_enum('RETURN_CODE')
BREAKPOINT_TYPE = parse_host_comm_enum('BREAKPOINT_TYPE')
INTERRUPT_SOURCE = parse_host_comm_enum('INTERRUPT_SOURCE')
ADC_CHAN_INDEX = parse_host_comm_enum('ADC_CHAN_INDEX')
ENERGY_BREAKPOINT_IMPL = parse_host_comm_enum('ENERGY_BREAKPOINT_IMPL')
CMP_REF = parse_host_comm_enum('CMP_REF')
UART_IDENTIFIER_USB = parse_host_comm_def('UART_IDENTIFIER_USB')

INTERRUPT_TYPE = parse_target_comm_enum('INTERRUPT_TYPE')

class StreamInterrupted(Exception):
    pass

class InterruptContext:
    def __init__(self, type, id, saved_vcap=None):
        self.type = type
        self.id = id
        self.saved_vcap = saved_vcap

class WispMonitor:
    VDD                                 = VDD # V
    CLK_FREQ                            = TIME_TIMER_FREQ # Hz
    CLK_PERIOD                          = 1.0 / CLK_FREQ # seconds
    CMP_VREF                            = 2.5
    CMP_BITS                            = 5

    def __init__(self):
        self.rxPkt = RxPkt()
        self.serial = serial.Serial(port=SERIAL_PORT, baudrate=CONFIG_USB_UART_BAUDRATE,
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
                
                if(self.rxPkt.identifier != UART_IDENTIFIER_USB):
                    # unknown identifier - reset state
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    raise Exception("packet construction failed: unknown identifier: " +
                            str(self.rxPkt.identifier) + " (exp " + str(UART_IDENTIFIER_USB) + ")")
                minBufLen -= 1
                self.rxPkt.constructState = CONSTRUCT_STATE_DESCRIPTOR
            elif(self.rxPkt.constructState == CONSTRUCT_STATE_DESCRIPTOR):
                self.rxPkt.descriptor = buf.pop(0) # get descriptor byte
                minBufLen -= 1
                self.rxPkt.constructState = CONSTRUCT_STATE_DATA_LEN
                continue
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
    #               descriptor = USB_CMD['SET_VCAP']
    #               adc = 2.2 * 2^12 / VDD = 2.2 * 4096 / 3.35 = 2689.91
    #               data = byteReverse(round(adc)) = byteReverse(2690)
    #                    = byteReverse(0x0A82)
    #                    = bytearray([0x82, 0x0A])
    def sendCmd(self, descriptor, data=[]):
        serialMsg = bytearray([UART_IDENTIFIER_USB, descriptor])
        serialMsg += bytearray([len(data)]) + bytearray(data)
        self.serial.write(serialMsg)

    def receive(self):
        if self.buildRxPkt(self.rcv_buf):
            pkt = { "descriptor" : self.rxPkt.descriptor }

            if self.rxPkt.descriptor == USB_RSP['RETURN_CODE']:
                pkt["code"] = self.rxPkt.data[0]

            elif self.rxPkt.descriptor == USB_RSP['TIME']:
                cycles = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                         (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0])
                pkt["time_sec"] = float(cycles) * self.CLK_PERIOD

            elif self.rxPkt.descriptor == USB_RSP['VOLTAGE']:
                adc_reading = (self.rxPkt.data[1] << 8) | self.rxPkt.data[0]
                pkt["voltage"] = self.adc_to_voltage(adc_reading)

            elif self.rxPkt.descriptor == USB_RSP['VOLTAGES']:
                offset = 0
                num_channels = self.rxPkt.data[offset]
                offset += 1
                pkt["voltages"] = []
                for i in range(num_channels):
                    adc_reading = (self.rxPkt.data[offset + 2 * i + 1] << 8) | \
                                   self.rxPkt.data[offset + 2 * i]
                    pkt["voltages"].append(self.adc_to_voltage(adc_reading))

            elif self.rxPkt.descriptor == USB_RSP['INTERRUPTED']:
                pkt["interrupt_type"] = key_lookup(INTERRUPT_TYPE, self.rxPkt.data[0])
                pkt["interrupt_id"] = self.rxPkt.data[1]
                saved_vcap_adc_reading = (self.rxPkt.data[3] << 8) | self.rxPkt.data[2]
                pkt["saved_vcap"] = self.adc_to_voltage(saved_vcap_adc_reading)

            elif self.rxPkt.descriptor == USB_RSP['WISP_MEMORY']:
                pkt["address"] = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                                 (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0] <<  0)
                pkt["value"] = self.rxPkt.data[4:]

            elif self.rxPkt.descriptor == USB_RSP['ADDRESS']:
                pkt["address"] = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                                 (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0] <<  0)

            elif self.rxPkt.descriptor == USB_RSP['SERIAL_ECHO']:
                pkt["value"] = self.rxPkt.data[0]

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
        if descriptor == USB_RSP['RETURN_CODE']: # this one is generic, so handle it here
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

    def cmp_ref_for_v(self, voltage):
        if voltage > 2.5:
            ref = "VCC"
        elif voltage > 2.0:
            ref = "VREF_2_5"
        elif voltage > 1.5:
            ref = "VREF_2_0"
        else:
            ref = "VREF_1_5"
        return ref

    def voltage_to_cmp(self, voltage):
        ref = self.cmp_ref_for_v(voltage)
        return int(voltage / (COMPARATOR_REF_VOLTAGE[ref] / 2**self.CMP_BITS)) - 1, ref

    def cmp_to_voltage(self, value, ref):
        return (float(value) + 1) * (COMPARATOR_REF_VOLTAGE[ref] / 2**CMP_BITS)

    def sense(self, channel):
        self.sendCmd(USB_CMD['SENSE'], data=[channel])
        reply = self.receive_reply(USB_RSP['VOLTAGE'])
        return reply["voltage"]

    def build_stream_cmd_data(self, channels):
        cmd_data = [len(channels)]
        for chan in channels:
            cmd_data.append(chan)
        return cmd_data

    def stream_begin(self, channels):
        self.stream_bytes = 0
        self.stream_start = time.time()
        self.sendCmd(USB_CMD['STREAM_BEGIN'], data=self.build_stream_cmd_data(channels))

    def stream_end(self, channels):
        self.sendCmd(USB_CMD['STREAM_END'], data=self.build_stream_cmd_data(channels))
        self.flush()

    def stream_datarate_kbps(self):
        return float(self.stream_bytes) / 1000 / (time.time() - self.stream_start)

    def charge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD['CHARGE'], data=cmd_data)
        reply = self.receive_reply(USB_RSP['VOLTAGE'])
        return reply["voltage"]

    def discharge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(USB_CMD['DISCHARGE'], data=cmd_data)
        reply = self.receive_reply(USB_RSP['VOLTAGE'])
        return reply["voltage"]

    def charge_cmp(self, target_voltage):
        target_voltage_cmp, ref = self.voltage_to_cmp(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_cmp) + [CMP_REF[ref]]
        self.sendCmd(USB_CMD['CHARGE_CMP'], data=cmd_data)
        self.receive_reply(USB_RSP['RETURN_CODE'])

    def discharge_cmp(self, target_voltage):
        target_voltage_cmp, ref = self.voltage_to_cmp(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_cmp) + [CMP_REF[ref]]
        self.sendCmd(USB_CMD['DISCHARGE_CMP'], data=cmd_data)
        self.receive_reply(USB_RSP['RETURN_CODE'])

    def enter_debug_mode(self):
        self.sendCmd(USB_CMD['ENTER_ACTIVE_DEBUG'])
        reply = self.receive_reply(USB_RSP['INTERRUPTED'])
        return reply["saved_vcap"]

    def exit_debug_mode(self):
        self.sendCmd(USB_CMD['EXIT_ACTIVE_DEBUG'])
        reply = self.receive_reply(USB_RSP['VOLTAGE'])
        return reply["voltage"]

    def interrupt(self):
        self.sendCmd(USB_CMD['INTERRUPT'])
        reply = self.receive_reply(USB_RSP['INTERRUPTED'])
        return reply["saved_vcap"]

    def break_at_vcap_level(self, level, impl):
        extra_args = []
        if impl == "adc":
            level = self.voltage_to_adc(level)
        elif impl == "cmp":
            level, ref = self.voltage_to_cmp(level)
            extra_args += [CMP_REF[ref]]
        else:
            raise Exception("Invalid energy breakpoint implementation method: " + impl)
        cmd_data = self.uint16_to_bytes(level) + [ENERGY_BREAKPOINT_IMPL[impl]] + extra_args
        self.sendCmd(USB_CMD['BREAK_AT_VCAP_LEVEL'], data=cmd_data)
        reply = self.receive_reply(USB_RSP['INTERRUPTED'])
        return reply["saved_vcap"]

    def breakpoint(self, type, idx, enable, energy_level=None):

        if energy_level is not None:
            energy_level_cmp, cmp_ref = self.voltage_to_cmp(energy_level)
        else:
            energy_level_cmp, cmp_ref = 0, "VCC"

        self.sendCmd(USB_CMD['BREAKPOINT'],
                     data=[BREAKPOINT_TYPE[type], idx] + \
                           self.uint16_to_bytes(energy_level_cmp) + \
                           [CMP_REF[cmp_ref], enable])
        self.receive_reply(USB_RSP['RETURN_CODE'])

    def wait_for_interrupt(self):
        pkt = self.receive_reply(USB_RSP['INTERRUPTED'])
        return InterruptContext(pkt["interrupt_type"], pkt["interrupt_id"], pkt["saved_vcap"])

    def get_interrupt_context(self, source):
        self.sendCmd(USB_CMD['GET_INTERRUPT_CONTEXT'], data=[INTERRUPT_SOURCE[source]])
        pkt = self.receive_reply(USB_RSP['INTERRUPTED'])
        return InterruptContext(pkt["interrupt_type"], pkt["interrupt_id"], pkt["saved_vcap"])

    def read_mem(self, addr, len):
        cmd_data = self.uint32_to_bytes(addr) + [len]
        self.sendCmd(USB_CMD['READ_MEM'], data=cmd_data)
        reply = self.receive_reply(USB_RSP['WISP_MEMORY'])
        return reply["address"], reply["value"]

    def write_mem(self, addr, value):
        cmd_data = self.uint32_to_bytes(addr) + [len(value)] + value
        self.sendCmd(USB_CMD['WRITE_MEM'], data=cmd_data)
        self.receive_reply(USB_RSP['RETURN_CODE'])

    def get_pc(self):
        self.sendCmd(USB_CMD['GET_WISP_PC'])
        reply = self.receive_reply(USB_RSP['ADDRESS'])
        return reply["address"]

    def cont_power(self, on):
        cmd_data = [on]
        self.sendCmd(USB_CMD['CONT_POWER'], data=cmd_data)
        self.receive_reply(USB_RSP['RETURN_CODE'])

    def serial_echo(self, value):
        cmd_data = [value]
        self.sendCmd(USB_CMD['SERIAL_ECHO'], data=cmd_data)
        reply = self.receive_reply(USB_RSP['SERIAL_ECHO'])
        return reply["value"]


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

