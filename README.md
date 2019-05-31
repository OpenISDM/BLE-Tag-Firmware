# BLE Tag Firmware

A customed firmware for our Bluetooth tag, which advertising packet contains several health measurments with a button press statement.

### Hardware

This repo was built with Nordic's nRF52 DK, running a NRF52832_XXAA CPU. Theoretically, any device with a nRF52832 chipset works well with this firmware. We have tested a few off-the-shelf devices and programmed with a J-Link, which all works perfectly.

### Preparing for programming the board

We recommed you to compile this program using [Seggar Embedded Studio](https://www.segger.com/products/development-tools/embedded-studio/), which is totally free and works very well with  ARM Processors. The NRF SDK we used for building this firmware was *nRF5 SDK 15.0.0*, which can be downloaded through [here](https://developer.nordicsemi.com/nRF5_SDK/). We also strongly recommend you to follow the Getting started guide from Nordic [here](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52-DK/Getting-Started), and you should be fully ready for development.

Make sure you have the correct board configurations for your board. Each board's pin layout can be very different, please always check your board's pin layout for setting up correctly. The board configuration files are placed under folder BOARD_CONFIG. 

| Config file | Supported board |
| ------ | ------ |
| pca10040_info_link.h | For tags from [InfoLink](https://item.taobao.com/item.htm?spm=a312a.7700824.w4002-16080569727.16.15cd6ec7jd8tHm&&id=585354834980) |
| pac10040_tag288.h | For tags from [RadioLand](https://item.taobao.com/item.htm?spm=a230r.1.14.118.49c76b1eRtZhrD&&id=572921086552&&ns=1&&abbucket=11) |

Simply place these files under the following directory from your SDK folder:

```sh
{SDK-root-folder}/components/boards/
```

...and remember to replace the imported config file of your project. For Segger Embedded Studio, the setting can be found inside the .emProject file from your project folder. Add a new board layout in *boards.h* and simply replace the config settings of *c_preprocessor_definitions* to connect your configuration file.

### Installation

Extract the SDK file, and navigate to /examples/ble_peripheral/ folder.

Clone this project under the folder above:

```sh
$ git clone https://github.com/OpenISDM/BLE-Tag-Firmware.git
```

Under each project, navigate through each folder base on the model number and softdevice of your board. For example, the nRF52 Development Kit should be navigate to

```sh
{project-folder}/pca10040/s132/
```

After selecting the correct configuration of your board, you will be able to select the preferred development environment and start your journey.

### Supported IDEs

There are multiple IDEs available for programming Nordic's CPU, and each of them are officially supported:

| IDE | Support |
| ------ | ------ |
| Seggar Embedded Studio | *Officially supported, tested* |
| IAR Embedded Workbench | *Officially supported* |
| KEIL uVision 4 | *Officially supported* |
| KEIL uVision 5 | *Officially supported* |
| GNU GCC | *Officially supported* |
| GNU GCC W/ Eclipse | *Unofficial, see [this](https://devzone.nordicsemi.com/tutorials/b/getting-started/posts/development-with-gcc-and-eclipse) post for setup* |

### Todos

 - Power consumption improving

License
----



