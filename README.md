# NUC140 Music Player

[![License](https://img.shields.io/badge/license-MIT-informational)](#license)
[![Platform](https://img.shields.io/badge/platform-Nuvoton%20NUC140-blue)](#hardware)
[![Codec](https://img.shields.io/badge/audio-IMA%20ADPCM%20mono%208kHz-blueviolet)](#audio-format)

An end-to-end, working music player for the Nuvoton Nu-LB-NUC140 learning
board. Reads IMA ADPCM `.wav` files from a FAT32 SD card via the onboard
reader and plays them through the WAU8822 codec to a single speaker on
the `CON3` header. Plug in the card, press Play, enjoy.

<!-- Replace with a short looped clip or photo of the board -->
<p align="center">
  <img src="docs/demo.gif" alt="NUC140 Music Player demo" width="640">
</p>

## Features

- Play, pause, previous, next
- Volume control with safe headroom mapping on WAU8822
- LCD track name display with 8.3 filename trimming
- Real-time progress bar normalized to song length and LCD width
- Debounced keypad input for single, reliable key events
- Auto-advance to the next track at end of file
- Dockerized GCC toolchain and one-command build

## Hardware

- Board: Nu-LB-NUC140 (ARM Cortex-M0)
- Audio codec: WAU8822 over I²C + I²S
- Storage: SD card (FAT32) in onboard slot
- Output: single speaker on `CON3`

## Audio format

- WAV container with IMA ADPCM (format `0x11`)
- Mono, 8 kHz sample rate
- 256-byte ADPCM blocks decoding to 504 PCM samples
- Files with `.WAV` extension and 8.3 filenames recommended

## Quickstart

Clone the repo, build the Docker toolchain, build the firmware, flash with
OpenOCD, copy WAVs to the SD card, press Play.

```sh
# 1) Build the toolchain image
docker build -t nuvoton-dev-env .

# 2) Build the firmware
./build.sh

# 3) Start OpenOCD
./openocd-init.sh
# In another terminal:
telnet localhost 4444 
reset halt
flash write_image erase /tmp/image.bin
reset run
exit
````

SD card prep: format as FAT32, drop your `.WAV` files in the root directory,
use 8.3 names like `TRACK01.WAV`, insert the card before boot.

## Controls

* Center: Play/Pause
* Left/Right: Previous/Next
* Up/Down: Volume Up/Down

Key scanning is debounced so each press registers exactly once.

## Build details

* Toolchain: ARM GCC inside Docker
* Flashing: OpenOCD scripts included
* Clocking: 8 kHz sample rate path configured in WAU8822 and I²S
* Double buffer: ping-pong PCM buffers fed by ADPCM decode
* UI: 16-char LCD lines show track name and a live progress bar

## Roadmap

* Optional 16 kHz mode
* SD subdirectory scanning and simple shuffle
* Stereo output path on WAU8822 mixers
* Simple on-device settings (volume persist, repeat mode)

## Troubleshooting

* No audio: verify `CON3` speaker wiring, WAU8822 I²C writes, and that files
  are IMA ADPCM.
* Glitches: use FAT32, keep filenames short, and prefer 8 kHz sources. Remember
  that the audio **MUST BE MONO!**
* Nothing listed: ensure `.WAV` extension and that files are in the SD card
  root.

## Resources

* Nuvoton Tools: [https://github.com/OpenNuvoton/Nuvoton\_Tools](https://github.com/OpenNuvoton/Nuvoton_Tools)
* Nu\_LB\_NUC140: [https://github.com/poyu39/Nu\_LB\_NUC140](https://github.com/poyu39/Nu_LB_NUC140)
* BONK-game-NUC140: [https://github.com/hoangdesu/BONK-game-NUC140](https://github.com/hoangdesu/BONK-game-NUC140)
* nuc140\_code: [https://github.com/rayee-github/nuc140\_code](https://github.com/rayee-github/nuc140_code)
* Nuc140-SD\_Audio: [https://github.com/vuongductuanktmt/Nuc140-SD\_Audio](https://github.com/vuongductuanktmt/Nuc140-SD_Audio)

## License

MIT. See `LICENSE`. 
