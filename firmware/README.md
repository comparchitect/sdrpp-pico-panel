# Pico Panel Firmware

Firmware for the Raspberry Pi Pico / Pico 2 W control surface used by the SDR++ Pico Panel module. It scans the full front panel (pots, encoders, switches, LEDs, and OLED) and speaks the ASCII protocol consumed by `src/pico_panel.cpp`.

## Hardware overview

| Function                          | Pico pins              |
|-----------------------------------|------------------------|
| I²C0 SDA / SCL (OLED)             | GP0 / GP1              |
| Frequency encoder A / B / push    | GP2 / GP3 / GP22       |
| Bandwidth encoder A / B / push    | GP4 / GP5 / GP13       |
| Switches 1–7 (active-low inputs)  | GP6…GP12               |
| LED outputs                       | GP14…GP20 (GP21 spare) |
| Potentiometers (VOL/ZOOM/SQL)     | GP26 / GP27 / GP28     |

Switch mapping and behaviors are described in the top-level README.

## Dependencies

The firmware uses the upstream Raspberry Pi Pico SDK and helper tools. Clone/install them beside this repo and export `PICO_SDK_PATH` before configuring:

```bash
git clone --depth=1 https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
export PICO_SDK_PATH=~/pico-sdk
# Optional utilities
git clone --depth=1 https://github.com/raspberrypi/picotool.git ~/picotool
git clone --depth=1 https://github.com/raspberrypi/pioasm.git ~/pioasm
```

## Building

```bash
cd firmware
mkdir -p build && cd build
cmake .. \
  -DPICO_BOARD=pico2_w \
  -DPICO_PLATFORM=rp2350 \
  -DPICO_SDK_PATH=$PICO_SDK_PATH
cmake --build . --target pico_panel
```

The resulting `pico_panel.uf2` can be dropped onto the Pico’s mass-storage device. Adjust the CMake options if you are targeting a different Pico board.

## Serial protocol

The firmware reports debounced events at ~50 Hz:

- `VOL:<0-4095>`, `ZOOM:<0-4095>`, `SQL:<0-4095>`
- `ENC_FREQ:<delta>`, `ENC_BW:<delta>`
- `BTN_FREQ:1`, `BTN_BW:1`
- `SWn:0/1` for switches 1–7 (active-high events)

It also accepts control messages from SDR++:

- `LED:<mask>` toggles the 7 LED outputs
- `OLED:<text>` renders multi-line status on the SH1106 display

See the repository README for the full mapping between messages and SDR++ behaviors.
