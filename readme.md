[![](https://dcbadge.vercel.app/api/server/3E8ca2dkcC)](https://discord.gg/3E8ca2dkcC)

![GitHub Downloads (all assets, all releases)](https://img.shields.io/github/downloads/bitaxeorg/esp-miner-multichip/total)


# ESP-Miner-Multichip

| Supported Targets | ESP32-S3 (BitAxe v2+) |
| ----------------- | --------------------- |

## Requires Python3.4 or later and pip

Install bitaxetool from pip. pip is included with Python 3.4 but if you need to install it check <https://pip.pypa.io/en/stable/installation/>

```
pip install --upgrade bitaxetool
```

## Hardware Required

This firmware is designed to run on a BitAxe v2+

If you do have a Bitaxe with no USB connectivity make sure to establish a serial connection with either a JTAG ESP-Prog device or a USB-to-UART bridge

## Preconfiguration

Starting with v2.0.0, the ESP-Miner-Multichip firmware requires some basic manufacturing data to be flashed in the NVS partition.

1. Download the esp-miner-multichip-factory.bin file from the release tab.
   Click [here](https://github.com/bitaxeorg/esp-miner-multichip/releases) for the release tab

2. Copy `config.cvs.example` to `config.cvs` and modify `asicfrequency`, `asicvoltage`, `asicmodel`, `devicemodel`, and `boardversion`

The following are recommendations but it is necessary that you do have all values in your `config.cvs`file to flash properly.

- recomended values for the Bitaxe 1366 (hex)

  ```
  key,type,encoding,value
  main,namespace,,
  asicfrequency,data,u16,485
  asicvoltage,data,u16,1200
  asicmodel,data,string,BM1366
  devicemodel,data,string,hex
  boardversion,data,string,303
  ```

## Flash

The bitaxetool includes all necessary library for flashing the binary file to the Bitaxe Hardware.

The bitaxetool requires a config.cvs preloaded file and the appropiate firmware.bin file in it's executed directory.

3. Flash with the bitaxetool

```
bitaxetool --config ./config.cvs --firmware ./esp-miner-multichip-factory.bin
```

## Build from source

Building the esp-miner-multichip firmware requires ESP-IDF v2.2 upwards and npm 10.2+

--> install VSCode extension for esp-idf and express install

in ./main/http_server/axe-os --> `npm install`--> `npm run build`

in ./ --> `idf.py build` --> this will create the build dir and you can use the `esp-miner-multichip.bin`and `wwww.bin` to update your Device.

## API
Bitaxe provides an API to expose actions and information.

For more details take a look at `main/http_server/http_server.c`.

Things that can be done are:
  
  - Get System Info
  - Get Swarm Info
  - Update Swarm
  - Swarm Options
  - System Restart Action
  - Update System Settings Action
  - System Options
  - Update OTA Firmware
  - Update OTA WWW
  - WebSocket

Some API examples in curl:
  ```bash
  # Get system information
  curl http://YOUR-BITAXE-IP/api/system/info
  ```
  ```bash
  # Get swarm information
  curl http://YOUR-BITAXE-IP/api/swarm/info
  ```
  ```bash
  # System restart action
  curl -X POST http://YOUR-BITAXE-IP/api/system/restart
  ```
