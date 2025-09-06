# NUC140 MUSIC PLAYER

Basic music player implementation for the Nuvoton Nu-LB-NUC140 learning board.
Reads IMA ADPCM mono 16bit @8kHz .wav files from an SD card, plays them on a
speaker attached to the `CON3` connector.

**Features:**

* Play/Pause
* Previous/Next
* Volume Up/Volume Down
* Display track name
* Track progress bar

## Dependencies

To flash the image, you need to have
[OpenOCD](https://github.com/openocd-org/openocd) installed and configured for
the NUC140 board.

## Building

### Toolchain

To build the docker toolchain containing the ARM GCC and necessary
dependencies, run:

```sh
docker build -t nuvoton-dev-env .
````

### Project

To build the music player project using the toolchain, run:

```sh
./build.sh
```

## OpenOCD

### Flashing

1. Run OpenOCD via:
  ```
  ./openocd-init.sh
  ```
1. Connect to the CLI server via:
  ```
  telnet localhost 4444
  ```
1. Put the board in `halt` mode:
  ```
  reset halt
  ```
1. Flash image via:
  ```
  flash write_image erase /tmp/image.bin
  ```
1. Run the board:
  ```
  reset run
  ```

## Resources

* [Nuvoton Tools](https://github.com/OpenNuvoton/Nuvoton_Tools)
* [Nu_LB_NUC140](https://github.com/poyu39/Nu_LB_NUC140)
* [BONK-game-NUC140](https://github.com/hoangdesu/BONK-game-NUC140)
* [nuc140_code](https://github.com/rayee-github/nuc140_code)
* [Nuc140-SD_Audio](https://github.com/vuongductuanktmt/Nuc140-SD_Audio)
  
