#!/usr/bin/env python3


cpu_freq = 16e6
max_freq = 10e3
resolution = 128
prescaller_bitsize = 14

impulsion_time = 10e-6

step = max_freq / resolution

speed_frequencies = [step * i for i in range(resolution)]
speed_delays = [(1/x - impulsion_time) for x in speed_frequencies if x != 0]


def find_prescaler(delay):
    for prescaler in range(prescaller_bitsize + 1):
        clock = cpu_freq / (2**prescaler)
        comparator = round(delay * clock)
        if comparator <= 255:
            return prescaler+1, comparator
    return None, None


comparators, prescalers = '\t0,', '\t0,'
for delay in speed_delays:
    pre, comp = find_prescaler(delay)
    comparators += f'\t{comp}, '
    prescalers += f'\t{pre}, '


with open('src/speedramp.hpp', 'w') as header:
    header.write('#include <inttypes.h>\n\nconst uint8_t prescaler[%d] = {\n%s};\n\n' % (resolution, prescalers))
    header.write('const uint8_t comparator[%d] = {\n%s};\n' % (resolution, comparators))
