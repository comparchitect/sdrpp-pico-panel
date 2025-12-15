# SDR++ Pico Panel Module

A misc module for SDR++ Brown (compatible with upstream SDR++) that interfaces with a Raspberry Pi Pico 2 W‑based control surface. The panel exposes:

- Three potentiometers (volume, zoom, squelch)
- Two rotary encoders (frequency tuning + step button, bandwidth + step button)
- Seven momentary switches with LEDs (frequency encoder push is separate)
- SH1106 128×64 OLED over I²C

A lightweight ASCII protocol runs over the Pico’s USB CDC link. The firmware scans the inputs, debounces them, and reports events, while the SDR++ module applies the requested actions and pushes LED/OLED updates back down to the Pico.

## Hardware Pinout

| Function                                   | Pico Pin |
|--------------------------------------------|----------|
| I²C0 SDA / SCL (OLED)                      | GP0 / GP1 |
| Frequency encoder A / B                    | GP2 / GP3 |
| Bandwidth encoder A / B                    | GP4 / GP5 |
| Switches 1–7 (active-low, pull-ups)        | GP6…GP12 |
| Bandwidth encoder push button              | GP13 |
| Frequency encoder push button              | GP22 |
| LED outputs (used)                         | GP14…GP20 |
| Spare LED output                           | GP21 |
| Potentiometers (volume / zoom / squelch)   | GP26 / GP27 / GP28 |

Switch mapping (LED-backed buttons on GP6–GP12):

1. Play / stop SDR
2. Cycle demod modes FM → WFM → AM
3. Cycle demod modes USB → LSB → DSB → CW
4. Cycle digital demod modes DSD ↔ OLD DSD
5. Toggle global mute
6. Toggle FT8/FT4 decoder module
7. Reserved / spare (leave unconnected if not needed)

Encoder pushes (no LEDs):

- Frequency encoder push (GP22) changes tuning step size (100 Hz → 1 kHz → … → 10 MHz).
- Bandwidth encoder push (GP13) toggles the bandwidth step size (100 Hz ↔ 1 kHz).


## Building the SDR++ Module

Point CMake to the `sdrpp_module.cmake` generated in your SDR++ (Brown or upstream) build tree.

```bash
git clone https://github.com/yourname/sdrpp-pico-panel.git
cd sdrpp-pico-panel
cmake -S . -B build \
  -DSDRPP_MODULE_CMAKE=/path/to/SDRPlusPlusBrown/build/sdrpp_module.cmake
cmake --build build -j$(nproc)
sudo cmake --install build
```

Enable the “Pico Panel” misc module inside SDR++ after installing.

## Building the Pico Firmware

The firmware lives in `firmware/pico_panel.cpp` and targets the Pico / Pico 2 W using the official Pico SDK.

### Firmware prerequisites

This repository no longer vendors the Raspberry Pi toolchain. Install the upstream dependencies beside this checkout, for example:

```bash
git clone --depth=1 https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
export PICO_SDK_PATH=~/pico-sdk
# Optional helper utilities (flash, USB console, etc.)
git clone --depth=1 https://github.com/raspberrypi/picotool.git ~/picotool
git clone --depth=1 https://github.com/raspberrypi/pioasm.git ~/pioasm
```

Make sure `PICO_SDK_PATH` points at your SDK directory before configuring.

```bash
cd firmware
mkdir -p build && cd build
cmake .. \
  -DPICO_BOARD=pico_w \
  -DPICO_SDK_PATH=/path/to/pico-sdk
cmake --build . --target pico_panel
```

Copy the generated `pico_panel.uf2` to the Pico’s USB mass-storage device to flash it.

## Serial Protocol (Pico → SDR++)

- `VOL:<0-4095>` – Volume pot (mapped to sink volume)
- `ZOOM:<0-4095>` – Waterfall zoom pot
- `SQL:<0-4095>` – Squelch pot
- `ENC_FREQ:<delta>` – Frequency encoder detent (± ticks)
- `ENC_BW:<delta>` – Bandwidth encoder detent
- `BTN_FREQ:1` – Frequency encoder push
- `SWn:0/1` – Switch state changes (n = 1…8, active-high events)

Messages are ASCII lines terminated by `\n`.

## Serial Protocol (SDR++ → Pico)

- `LED:<mask>` – 8-bit LED bitmap (bit0 = switch 1 LED … bit7 = switch 8 LED, bit5 unused)
- `OLED:<text>` – Text rendered on the SH1106 display (currently the tuned frequency)

SDR++ proactively sends LED masks any time a tracked state changes and updates the OLED whenever the tuned frequency changes noticeably.

## Current Status

- Volume, zoom, and squelch pots map directly to SDR++ controls.
- Frequency encoder tunes the active VFO using the selected step (100 Hz → 10 MHz) and updates the OLED frequency readout.
- Bandwidth encoder adjusts the demodulator bandwidth with 100 Hz / 1 kHz steps.
- Switches trigger play/pause, demodulator rotations, DSD selection, mute, and FT8/FT4 toggling; LEDs follow the actual SDR++ state.
- Firmware debounces inputs, handles LED masks, and renders the OLED text.

Future ideas:

- Additional OLED layouts (e.g., demod/mode indicators)
- Configurable mappings (JSON-driven) instead of fixed bindings
- Optional audio streaming back to the Pico using the remaining GPIO resources

## License

All original code in this repository is released under the MIT License (see `LICENSE`). The Raspberry Pi Pico SDK, picotool, and pioasm are not bundled—please obtain them from their respective upstream repositories and comply with their licenses when building the firmware.
