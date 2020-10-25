#!/usr/bin/env python3


cpu_freq = 16e6
max_freq = 10e3
resolution = 1024
prescaller_bitsize = 10

impulsion_time = 10e-6

step = max_freq / resolution

speed_frequencies = [step * i for i in range(resolution)]
speed_delays = [(1/x - impulsion_time) for x in speed_frequencies if x != 0]


atmega_prescalers = {
    0: 1,
    3: 2,
    6: 3,
    8: 4,
    10: 5,
}


def find_prescaler(delay):
    for prescaler in atmega_prescalers:
        clock = cpu_freq / (2**prescaler)
        comparator = round(delay * clock)
        if comparator <= 255:
            return atmega_prescalers[prescaler], comparator
    return 0, 0


comparators, prescalers = '\t0,', '\t0,'
for delay in speed_delays:
    pre, comp = find_prescaler(delay)
    comparators += f'\t{comp}, '
    prescalers += f'\t{pre}, '


with open('src/speedramp.hpp', 'w') as header:
    header.write('#include <inttypes.h>\n#include <avr/pgmspace.h>\n\nconst PROGMEM uint8_t prescaler[%d] = {\n%s};\n\n' % (resolution, prescalers))
    header.write('const PROGMEM uint8_t comparator[%d] = {\n%s};\n' % (resolution, comparators))
