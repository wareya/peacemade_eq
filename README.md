# Peacemade EQ

Parametric four-filter EQ firmware for the Teensy 4.1 with Audio Shield. Uses Teensy Audio Library, stores filters to EEPROM, and can be configured over USB MIDI.

Uses the Arduino IDE. Settings:

![image](https://github.com/wareya/peacemade_eq/assets/585488/890fc8d8-3558-434a-95a0-24c7417c1981)

(24 MHz, Serial + MIDI + Audio)

Running at more than 24 MHz may result in audible interference noise. Also, be sure to use a high-quality powered USB hub for this and any other audio peripherals attached to your computer.

Works on power-only USB too, but can't be reconfigured in this state. Configuration will be remembered from the last time it was plugged into a PC.

## Configuration

The filters are configured over USB MIDI. Yes, really. You can use something like MIDI Tools (https://mountainutilities.eu/miditools) to change the filterse in realtime as the EQ is running.

When first opening it, set Controller 63 to 1 to get an update of what the current value is on all controllers. The meaning of each controller is as follows:

```
0, 3, 6, 9 : filter frequency, coarse adjustment (10 hz to 24 khz, logarithmic)
1, 4, 7, 10 : filter gain, coarse adjustment (-18.0 to +18.0db, linear in db, logarithmic in amplitude)
2, 5, 8, 11 : filter resonance, coarse adjustment (0.333.... to 33.333..., logarithmic)

32, 35, etc : fine versions of the above coarse adjustments

63 : set to 1 to get an update from the device on what the current value of each controller is
64 ~ 67 : filter type (0 = null, 1 = low pass, 2 = high pass, 3 = low shelf, 4 = high shelf, 5 = band pass, 5 = notch, 7 = peak)
94 : 0 = analog output, 1 = digital output only
95 : input level (0 to 15)
31 : output level (0 to 127)
```

## License

Code is licensed under the Apache 2.0 license. 3D models are licensed under the CC0 license: [https://creativecommons.org/licenses/zero](https://creativecommons.org/publicdomain/zero/1.0/legalcode)

## TODO

Everything works, but latency is higher than it needs to be (128 samples instead of arbitrarily low). This is because of bugs in the Teensy Audio Library. It would be nice to get rid of Teensy Audio Library and talk to the DAC/ADC directly over I2S or SPI or whatever it uses.

It would also be nice to support arbitrarily many filters (at least up to 7, as the current control layout supports).

## Case (3d model)

![IMG_20240124_055720](https://github.com/wareya/peacemade_eq/assets/585488/960d4a68-fc35-4628-afe4-f003e2c72f6e)

![IMG_20240125_010053](https://github.com/wareya/peacemade_eq/assets/585488/04d9a487-6b3c-49de-8565-791492847fe2)

## Approximate cost

The Teensy 4.1 costs about 30 dollars. The Audio Shield costs about 15 dollars. Add in extra money for soldering work, 3d printing, and buying or scavanging a line-in plug, and this project costs more than 45 dollars to build.
