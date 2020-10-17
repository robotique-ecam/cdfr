### Firmwares

#### ATTiny-85 Based

Change MCU Frequency from 1MHz (default) to approx 16MHz on internal oscilator

```
avrdude -P /dev/cu.usbserial-1420 -c avrisp -b 19200 -p attiny85 -U lfuse:w:0xF1:m
```

#### ATMega328p Based

Change MCU Frequency from 1MHz (default) to approx 8MHz on internal oscilator

```
avrdude -P /dev/cu.usbserial-1410 -c avrisp -b 19200 -p m328p -U lfuse:w:0xE2:m
```

#### Platformio

In some cases (m328p), using Platformio's builtin upload function, the flashing process will fail with reported bytes mismatch. Give a try to the command bellow which adds reset / clear behavior before uploading.

```
pio run --target program
```
