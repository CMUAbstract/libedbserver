# WISP Monitor Python Class

import serial
import math
import time
import re
import sys
from binascii import hexlify

import env
from delayed_keyboard_interrupt import *
from header_parser import Header

SERIAL_PORT                         = '/dev/ttyUSB0'

UART_LOG_FILE   = open("uart.log", "w")

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

host_comm_header = Header(env.HOST_COMM_HEADER,
    enums=[
        'USB_CMD',
        'USB_RSP',
        'RETURN_CODE',
        'BREAKPOINT_TYPE',
        'INTERRUPT_SOURCE',
        'ADC_CHAN_INDEX',
        'ENERGY_BREAKPOINT_IMPL',
        'CMP_REF',
        'STREAM',
        'RF_EVENT'
    ],
    numeric_macros=[
        'UART_IDENTIFIER_USB'
    ])

target_comm_header = Header(env.TARGET_COMM_HEADER,
    enums=['INTERRUPT_TYPE'])

config_header = Header(env.CONFIG_HEADER,
    string_macros=[
        'CONFIG_TIMELOG_TIMER_SOURCE'
    ],
    numeric_macros=[
        'CONFIG_USB_UART_BAUDRATE',
        'CONFIG_XT1_FREQ',
        'CONFIG_REFO_FREQ',
        'CONFIG_DCOCLKDIV_FREQ'
    ])

class StreamInterrupted(Exception):
    pass

class InterruptContext:
    def __init__(self, type, id, saved_vcap=None):
        self.type = type
        self.id = id
        self.saved_vcap = saved_vcap

class StreamDataPoint:
    def __init__(self, timestamp_sec, value_set):
        self.timestamp_sec = timestamp_sec
        self.value_set = value_set 

class StreamDecodeException(Exception):
    def __init__(self, msg=""):
        self.message = msg

class WispMonitor:
    VDD                                 = VDD # V
    CMP_VREF                            = 2.5
    CMP_BITS                            = 5

    def __init__(self):
        self.rxPkt = RxPkt()
        self.serial = serial.Serial(port=SERIAL_PORT,
                                    baudrate=config_header.macros['CONFIG_USB_UART_BAUDRATE'],
                                    timeout=1)
        self.serial.close()
        self.serial.open()

        self.rcv_buf = bytearray()
        self.stream_bytes = 0

        self.stream_decoders = {
                "VCAP": self.decode_adc_value,
                "VBOOST": self.decode_adc_value,
                "VREG": self.decode_adc_value,
                "VRECT": self.decode_adc_value,
                "VINJ": self.decode_adc_value,
                "RF_EVENTS": self.decode_rf_event_value,
        }

        # Cutting corners a bit: assume that ACLK is sourced either from XT1 or REFO
        timelog_source = config_header.macros['CONFIG_TIMELOG_TIMER_SOURCE']
        if timelog_source == 'TASSEL__ACLK':
            assert config_header.macros['CONFIG_XT1_FREQ'] == config_header.macros['CONFIG_REFO_FREQ']
            self.CLK_FREQ = config_header.macros['CONFIG_XT1_FREQ']
        elif timelog_source == 'TASSEL__SMCLK':
            self.CLK_FREQ = config_header.macros['CONFIG_DCOCLKDIV_FREQ']
        self.CLK_PERIOD = 1.0 / self.CLK_FREQ # seconds
        
    def destroy(self):
        self.serial.close()
    
    def buildRxPkt(self, buf):
        """Parses packet header and returns whether it is ready or not"""

        minBufLen = len(buf)
        while minBufLen > 0:
            if(self.rxPkt.constructState == CONSTRUCT_STATE_IDENTIFIER):
                self.rxPkt.identifier = buf.pop(0) # get the identifier byte
                minBufLen -= 1
                
                if(self.rxPkt.identifier != host_comm_header.macros['UART_IDENTIFIER_USB']):
                    # unknown identifier - reset state
                    self.rxPkt.constructState = CONSTRUCT_STATE_IDENTIFIER
                    print >>sys.stderr, "packet construction failed: unknown identifier: ", \
                            self.rxPkt.identifier, " (exp ", \
                            str(host_comm_header.macros['UART_IDENTIFIER_USB']), ")"
                    continue
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
    #               descriptor = host_comm_header.enums['USB_CMD']['SET_VCAP']
    #               adc = 2.2 * 2^12 / VDD = 2.2 * 4096 / 3.35 = 2689.91
    #               data = byteReverse(round(adc)) = byteReverse(2690)
    #                    = byteReverse(0x0A82)
    #                    = bytearray([0x82, 0x0A])
    def sendCmd(self, descriptor, data=[]):
        serialMsg = bytearray([host_comm_header.macros['UART_IDENTIFIER_USB'], descriptor])
        serialMsg += bytearray([len(data)]) + bytearray(data)
        self.serial.write(serialMsg)

    def receive(self):
        if self.buildRxPkt(self.rcv_buf):
            pkt = { "descriptor" : self.rxPkt.descriptor }

            if self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['RETURN_CODE']:
                pkt["code"] = self.rxPkt.data[0]

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['TIME']:
                cycles = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                         (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0])
                pkt["time_sec"] = float(cycles) * self.CLK_PERIOD

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['VOLTAGE']:
                adc_reading = (self.rxPkt.data[1] << 8) | self.rxPkt.data[0]
                pkt["voltage"] = self.adc_to_voltage(adc_reading)

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['VOLTAGES']:
                offset = 0
                num_channels = self.rxPkt.data[offset]
                offset += 1
                pkt["voltages"] = []
                for i in range(num_channels):
                    adc_reading = (self.rxPkt.data[offset + 2 * i + 1] << 8) | \
                                   self.rxPkt.data[offset + 2 * i]
                    pkt["voltages"].append(self.adc_to_voltage(adc_reading))

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['INTERRUPTED']:
                pkt["interrupt_type"] = key_lookup(target_comm_header.enums['INTERRUPT_TYPE'], self.rxPkt.data[0])
                pkt["interrupt_id"] = self.rxPkt.data[1]
                saved_vcap_adc_reading = (self.rxPkt.data[3] << 8) | self.rxPkt.data[2]
                pkt["saved_vcap"] = self.adc_to_voltage(saved_vcap_adc_reading)

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['WISP_MEMORY']:
                pkt["address"] = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                                 (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0] <<  0)
                pkt["value"] = self.rxPkt.data[4:]

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['ADDRESS']:
                pkt["address"] = (self.rxPkt.data[3] << 24) | (self.rxPkt.data[2] << 16) | \
                                 (self.rxPkt.data[1] <<  8) | (self.rxPkt.data[0] <<  0)

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['SERIAL_ECHO']:
                pkt["value"] = self.rxPkt.data[0]

            elif self.rxPkt.descriptor == host_comm_header.enums['USB_RSP']['STREAM_DATA']:
                FIELD_LEN_STREAMS = 1
                FIELD_LEN_TIMESTAMP = 4

                offset = 0

                data_points = []

                if offset + FIELD_LEN_STREAMS <= len(self.rxPkt.data):

                    pkt_streams = self.rxPkt.data[offset]
                    #print "pkt_streams=0x%08x" % pkt_streams
                    offset += 1
                    offset += 1 # padding


                    while offset < len(self.rxPkt.data):
                        # invalid pkt, but best we can do here is not fail
                        if offset + FIELD_LEN_TIMESTAMP > len(self.rxPkt.data):
                            print >>sys.stderr, "WARNING: corrupt STREAM_DATA pkt: timestamp field"
                            break

                        timestamp_cycles = (self.rxPkt.data[offset + 3] << 24) | \
                                           (self.rxPkt.data[offset + 2] << 16) | \
                                           (self.rxPkt.data[offset + 1] <<  8) | \
                                           (self.rxPkt.data[offset + 0] <<  0)
                        offset += 4

                        timestamp_sec = float(timestamp_cycles) * self.CLK_PERIOD

                        try:
                            # Decode the pkt into a dictionary: stream->value.
                            # Order is implied by the bit index of each stream as defined by the enum
                            value_set = {}
                            for stream in host_comm_header.enums['STREAM']:
                                if pkt_streams & host_comm_header.enums['STREAM'][stream]:
                                    value, length = self.stream_decoders[stream](self.rxPkt.data, offset)
                                    offset += length
                                    value_set[stream] = value

                            data_points.append(StreamDataPoint(timestamp_sec, value_set))
                        except StreamDecodeException as e:
                            # invalid pkt, best we can do here is not fail
                            print >>sys.stderr, "WARNING: corrupt STREAM_DATA pkt: value field: " + \
                                    e.message
                            break
                else:
                    # invalid pkt, best we can do here is not fail
                    print >>sys.stderr, "WARNING: corrupt STREAM_DATA pkt: streams field"

                pkt["data_points"] = data_points

            return pkt

        newData = self.serial.read()

        if len(newData) > 0:
            newBytes = bytearray(newData)
            UART_LOG_FILE.write(newBytes)
            UART_LOG_FILE.flush()
            self.rcv_buf.extend(newBytes)
            self.stream_bytes += len(newData)
        return None

    def receive_reply(self, descriptor):
        reply = None
        while reply is None:
            while reply is None:
                reply = self.receive()
            if reply["descriptor"] != descriptor:
                print >>sys.stderr, "unexpected reply: ", \
                        reply["descriptor"], "(exp ", str(descriptor), ")"
                reply = None
                continue
        if descriptor == host_comm_header.enums['USB_RSP']['RETURN_CODE']: # this one is generic, so handle it here
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

    def decode_adc_value(self, bytes, offset):
        FIELD_LEN = 2
        #if offset + FIELD_LEN > len(bytes):
        #    raise StreamDecodeException()
        adc_value = (bytes[offset + 1] << 8) | (bytes[offset] << 0)
        length = 2
        voltage = self.adc_to_voltage(adc_value)
        return voltage, length

    def decode_rf_event_value(self, bytes, offset):
        FIELD_LEN = 2
        if offset + FIELD_LEN > len(bytes):
            raise StreamDecodeException("not enough bytes")
        rf_event_id = (bytes[offset + 1] << 8) | (bytes[offset] << 0)
        #print "rf_event_id = 0x%08x" % rf_event_id
        rf_event = key_lookup(host_comm_header.enums['RF_EVENT'], rf_event_id)
        if rf_event is None:
            raise StreamDecodeException("invalid event id: " + "0x%04x" % rf_event_id)
        length = 2 # sizeof(rf_event_t.id field + padding in the struct)
        return rf_event, length

    def sense(self, channel):
        self.sendCmd(host_comm_header.enums['USB_CMD']['SENSE'], data=[channel])
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['VOLTAGE'])
        return reply["voltage"]

    def stream_begin(self, streams_bitmask):
        self.stream_bytes = 0
        self.stream_start = time.time()
        self.sendCmd(host_comm_header.enums['USB_CMD']['STREAM_BEGIN'], data=[streams_bitmask])

    def stream_end(self, streams_bitmask):
        self.sendCmd(host_comm_header.enums['USB_CMD']['STREAM_END'], data=[streams_bitmask])
        self.flush()

    def stream_datarate_kbps(self):
        return float(self.stream_bytes) / 1000 / (time.time() - self.stream_start)

    def charge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(host_comm_header.enums['USB_CMD']['CHARGE'], data=cmd_data)
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['VOLTAGE'])
        return reply["voltage"]

    def discharge(self, target_voltage):
        target_voltage_adc = self.voltage_to_adc(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_adc)
        self.sendCmd(host_comm_header.enums['USB_CMD']['DISCHARGE'], data=cmd_data)
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['VOLTAGE'])
        return reply["voltage"]

    def charge_cmp(self, target_voltage):
        target_voltage_cmp, ref = self.voltage_to_cmp(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_cmp) + [host_comm_header.enums['CMP_REF'][ref]]
        self.sendCmd(host_comm_header.enums['USB_CMD']['CHARGE_CMP'], data=cmd_data)
        self.receive_reply(host_comm_header.enums['USB_RSP']['RETURN_CODE'])

    def discharge_cmp(self, target_voltage):
        target_voltage_cmp, ref = self.voltage_to_cmp(target_voltage)
        cmd_data = self.uint16_to_bytes(target_voltage_cmp) + [host_comm_header.enums['CMP_REF'][ref]]
        self.sendCmd(host_comm_header.enums['USB_CMD']['DISCHARGE_CMP'], data=cmd_data)
        self.receive_reply(host_comm_header.enums['USB_RSP']['RETURN_CODE'])

    def enter_debug_mode(self):
        self.sendCmd(host_comm_header.enums['USB_CMD']['ENTER_ACTIVE_DEBUG'])
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['INTERRUPTED'])
        return reply["saved_vcap"]

    def exit_debug_mode(self):
        self.sendCmd(host_comm_header.enums['USB_CMD']['EXIT_ACTIVE_DEBUG'])
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['VOLTAGE'])
        return reply["voltage"]

    def interrupt(self):
        self.sendCmd(host_comm_header.enums['USB_CMD']['INTERRUPT'])
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['INTERRUPTED'])
        return reply["saved_vcap"]

    def break_at_vcap_level(self, level, impl):
        extra_args = []
        if impl == "adc":
            level = self.voltage_to_adc(level)
        elif impl == "cmp":
            level, ref = self.voltage_to_cmp(level)
            extra_args += [host_comm_header.enums['CMP_REF'][ref]]
        else:
            raise Exception("Invalid energy breakpoint implementation method: " + impl)
        cmd_data = self.uint16_to_bytes(level) + [host_comm_header.enums['ENERGY_BREAKPOINT_IMPL'][impl]] + extra_args
        self.sendCmd(host_comm_header.enums['USB_CMD']['BREAK_AT_VCAP_LEVEL'], data=cmd_data)
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['INTERRUPTED'])
        return reply["saved_vcap"]

    def breakpoint(self, type, idx, enable, energy_level=None):

        if energy_level is not None:
            energy_level_cmp, cmp_ref = self.voltage_to_cmp(energy_level)
        else:
            energy_level_cmp, cmp_ref = 0, "VCC"

        self.sendCmd(host_comm_header.enums['USB_CMD']['BREAKPOINT'],
                     data=[host_comm_header.enums['BREAKPOINT_TYPE'][type], idx] + \
                           self.uint16_to_bytes(energy_level_cmp) + \
                           [host_comm_header.enums['CMP_REF'][cmp_ref], enable])
        self.receive_reply(host_comm_header.enums['USB_RSP']['RETURN_CODE'])

    def wait_for_interrupt(self):
        pkt = self.receive_reply(host_comm_header.enums['USB_RSP']['INTERRUPTED'])
        return InterruptContext(pkt["interrupt_type"], pkt["interrupt_id"], pkt["saved_vcap"])

    def get_interrupt_context(self, source):
        self.sendCmd(host_comm_header.enums['USB_CMD']['GET_INTERRUPT_CONTEXT'], data=[host_comm_header.enums['INTERRUPT_SOURCE'][source]])
        pkt = self.receive_reply(host_comm_header.enums['USB_RSP']['INTERRUPTED'])
        return InterruptContext(pkt["interrupt_type"], pkt["interrupt_id"], pkt["saved_vcap"])

    def read_mem(self, addr, len):
        cmd_data = self.uint32_to_bytes(addr) + [len]
        self.sendCmd(host_comm_header.enums['USB_CMD']['READ_MEM'], data=cmd_data)
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['WISP_MEMORY'])
        return reply["address"], reply["value"]

    def write_mem(self, addr, value):
        cmd_data = self.uint32_to_bytes(addr) + [len(value)] + value
        self.sendCmd(host_comm_header.enums['USB_CMD']['WRITE_MEM'], data=cmd_data)
        self.receive_reply(host_comm_header.enums['USB_RSP']['RETURN_CODE'])

    def get_pc(self):
        self.sendCmd(host_comm_header.enums['USB_CMD']['GET_WISP_PC'])
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['ADDRESS'])
        return reply["address"]

    def cont_power(self, on):
        cmd_data = [on]
        self.sendCmd(host_comm_header.enums['USB_CMD']['CONT_POWER'], data=cmd_data)
        self.receive_reply(host_comm_header.enums['USB_RSP']['RETURN_CODE'])

    def serial_echo(self, value):
        cmd_data = [value]
        self.sendCmd(host_comm_header.enums['USB_CMD']['SERIAL_ECHO'], data=cmd_data)
        reply = self.receive_reply(host_comm_header.enums['USB_RSP']['SERIAL_ECHO'])
        return reply["value"]

    def stream(self, streams, duration_sec=None, out_file=None, silent=True):
        streams_bitmask = 0x0
        for stream in streams:
            streams_bitmask |= host_comm_header.enums['STREAM'][stream]

        streaming = True
        self.stream_begin(streams_bitmask)

        if not silent:
            print "Logging... Ctrl-C to stop"

        start_time_sec = None
        timestamp_sec = 0
        num_samples = 0

        def format_voltage(voltage):
            return "%f" % voltage
        def format_rf_event(rf_event):
            return rf_event # a string

        stream_formaters = {
            "VCAP": format_voltage,
            "VBOOST": format_voltage,
            "VREG": format_voltage,
            "VRECT": format_voltage,
            "VINJ": format_voltage,
            "RF_EVENTS": format_rf_event,
        }
        
        with DelayedKeyboardInterrupt(): # prevent partial lines
            out_file.write("timestamp_sec," + ",".join(streams) + "\n")
        
        try:
            while duration_sec is None or timestamp_sec < duration_sec:
                pkt = self.receive_reply(host_comm_header.enums['USB_RSP']['STREAM_DATA'])

                for data_point in pkt["data_points"]:

                    if start_time_sec is None: # first time data - store it for reference
                        start_time_sec = data_point.timestamp_sec
                    timestamp_sec = data_point.timestamp_sec - start_time_sec

                    line = "%f" % timestamp_sec
                    for stream in streams:
                        # a column per requested stream, so may have blanks on some rows
                        line += ","
                        if stream in data_point.value_set:
                            line += stream_formaters[stream](data_point.value_set[stream])
                    line += "\n"

                    with DelayedKeyboardInterrupt(): # prevent partial lines
                        out_file.write(line)

                    num_samples += 1

                if not silent:
                    print "\r%.2f KB/s" % self.stream_datarate_kbps()

        finally:
            self.stream_end(streams_bitmask)

            # clear partial packets from the input buf (flush a few times just
            # in case data is backed up in the buffer on the device side)
            for i in range(1, 2):
                time.sleep(0.25)
                self.serial.flushInput()

        if not silent:
            print "%d samples in %f seconds (target time)" % (num_samples, timestamp_sec)

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

