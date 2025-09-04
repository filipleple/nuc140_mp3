# NUC140 MP3 PLAYER

## Requirements

* [Nuvoton NuMicro ICP Programming Tool 3.20](https://www.nuvoton.com/resource-download.jsp?tp_GUID=SW1720200221181328)
* [Keil uVision v5](https://www.keil.com/demo/eval/arm.htm)
* [SDK/BSP](https://drive.google.com/file/d/1ZAxvx0Aq6V41U7cmGF6-J-SBrkuUI4_I/view?usp=sharing)

## Resources

### Meta

* [Nuvoton Tools](https://github.com/OpenNuvoton/Nuvoton_Tools)

### Code

* [Nu_LB_NUC140](https://github.com/poyu39/Nu_LB_NUC140)
* [BONK-game-NUC140](https://github.com/hoangdesu/BONK-game-NUC140)
* [nuc140_code](https://github.com/rayee-github/nuc140_code)
* [Nuc140-SD_Audio](https://github.com/vuongductuanktmt/Nuc140-SD_Audio)

### Tutorial

* [Creating New Project from Scratch - Nuvoton NU-LB-NUC140 Board](https://www.youtube.com/watch?v=OubVJVUpvIk)
* [Nuvoton NUC140 Initial Setup tutorial](https://www.youtube.com/watch?v=c-3kSXNbJ78)

## Flashing

### OpenOCD

1. Run OpenOCD via
  ```
  ./openocd-init.sh
  ```
1. Connect to the CLI server via
  ```
  telnet localhost 4444
  ```
1. Flash image via
  ```
  flash write_image erase /tmp/image.bin
  ```

## Troubleshooting

* Multiple build errors related to `inline` definitions:
  - Set C standard to gnu99
* Missing compiler:
  - Choose ARM v6 for project
