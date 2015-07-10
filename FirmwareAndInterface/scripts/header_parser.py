import re
from collections import OrderedDict

def number_from_string(s):
    if s.startswith('0x'):
        val = int(s, 16)
    else:
        val = int(s)
    return val

class Header:
    """Parses C headers and translates the given enums and macros into dicts"""

    def __init__(self, header_file, enums=[], numeric_macros=[], string_macros=[]):
        self.header_file = header_file

        self.enums = {}
        self.macros = {}

        for enum in enums:
            self.enums[enum] = self.parse_enum(enum)
        for macro in numeric_macros:
            self.macros[macro] = self.parse_def(macro, cast_func=number_from_string)
        for macro in string_macros:
            self.macros[macro] = self.parse_def(macro, cast_func=str)

    def parse_enum(self, prefix):
        enum_dict = OrderedDict()
        for line in open(self.header_file):
            m = re.match(r'^\s*' + prefix + r'_(?P<name>\w+)\s*=\s*(?P<value>[0-9xA-F]+)', line)
            if m is None:
                continue
            name = m.group('name')
            enum_dict[name] = number_from_string(m.group('value'))
        return enum_dict

    def parse_def(self, name, cast_func):
        for line in open(self.header_file):
            m = re.match(r'^\s*#define\s+' + name + '\s+(?P<value>\w+)\s*', line)
            if m is None:
                continue
            return cast_func(m.group('value'))
        raise Exception("Macro '" + name + "' not found in '" + self.header_file + "'")
