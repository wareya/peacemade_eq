# Peacemade EQ

Parametric four-filter EQ firmware for the Teensy 4.1 with Audio Shield. Uses Teensy Audio Library, stores filters to EEPROM, and can be configured over USB MIDI.

Uses the Arduino IDE. Settings:

![image](https://github.com/wareya/peacemade_eq/assets/585488/890fc8d8-3558-434a-95a0-24c7417c1981)

(24 MHz, Serial + MIDI + Audio)

Running at more than 24 MHz may result in audible interference noise. Also, be sure to use a high-quality powered USB hub for this and any other audio peripherals attached to your computer.

Works on power-only USB too, but can't be reconfigured in this state. Configuration will be remembered from the last time it was plugged into a PC.

## License

Apache 2.0
