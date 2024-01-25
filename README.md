# Peacemade EQ

Parametric four-filter EQ firmware for the Teensy 4.1 with Audio Shield. Uses Teensy Audio Library, stores filters to EEPROM, and can be configured over USB MIDI.

Uses the Arduino IDE. Settings:

![image](https://github.com/wareya/peacemade_eq/assets/585488/890fc8d8-3558-434a-95a0-24c7417c1981)

(24 MHz, Serial + MIDI + Audio)

Running at more than 24 MHz may result in audible interference noise. Also, be sure to use a high-quality powered USB hub for this and any other audio peripherals attached to your computer.

Works on power-only USB too, but can't be reconfigured in this state. Configuration will be remembered from the last time it was plugged into a PC.

## License

Apache 2.0

## TODO

Everything works, but latency is higher than it needs to be (128 samples instead of arbitrarily low). This is because of bugs in the Teensy Audio Library. It would be nice to get rid of Teensy Audio Library and talk to the DAC/ADC directly over I2S or SPI or whatever it uses.

## Case (3d model)

![IMG_20240124_055720](https://github.com/wareya/peacemade_eq/assets/585488/960d4a68-fc35-4628-afe4-f003e2c72f6e)

![IMG_20240125_010053](https://github.com/wareya/peacemade_eq/assets/585488/04d9a487-6b3c-49de-8565-791492847fe2)
