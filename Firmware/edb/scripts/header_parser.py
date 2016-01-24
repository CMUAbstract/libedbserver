import re
from collections import OrderedDict

def number_from_string(s):
    m = re.match(r'(?P<hex>0x)?(?P<val>[0-9a-fA-F]+)[uU]?([lL]+)?', s)
    if m is None:
        raise Exception("Invalid numeric value: " + s)
    if m.group('hex') is not None and len(m.group('hex')) > 0:
        val = int('0x' + m.group('val'), 16)
    else:
        val = int(m.group('val'))
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
            m = re.match(r'^\s*' + prefix + r'_(?P<name>\w+)\s*=\s*(?P<value>[0-9xa-fA-F]+)', line)
            if m is None:
                continue
            name = m.group('name')
            enum_dict[name] = number_from_string(m.group('value'))
        return enum_dict

    def parse_def(self, name, cast_func):
        for line in open(self.header_file):
            m = re.match(r'^\s*#define\s+' + name + '\s+(?P<value>\w+)?\s*', line)
            if m is None:
                continue
            val = m.group('value')
            if val is None:
                return True # defined macros without value converted to bool
            else:
                return cast_func(val)
        return None

    def as_dict(self):
        return {"enums": self.enums, "macros": self.macros}
