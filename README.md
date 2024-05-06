# Spaceballs - The Switch!

Convert the [Concord Aerospace](https://www.concordaerospace.com/)'s 
[Ludicrous Speed Panel](https://www.concordaerospace.com/products/ludicrous-speed-control-panel) 
into a 5 switch Zigbee on/off switch. Only tested with Home Assistant. Switches 1-4 will be turned
on when the potentiometer is rotated (you may need to input your own calibration values though),
eg. plaid will have switched 1-4 in ON state. GO switch is switch number 5 and operates independently.

Requires ESP32-C6 and ESP-IDF 5.2. Tested with WeAct Studio ESP32-C6-MINI.

## Connecting hardware

Connect POT 1 to 3.3 VCC, POT 2 to pin 8, POT 3 to GND. Connect GO button to 3.3 VCC
and pin 20.

## Resetting device

Jumper 3V3 to pin 23 and reset the device.

## Building and flashing

Plug in your ESP32C6 to your computer (press BOOT, hold it and press RESET, then release BOOT) and replace `com4` with the
COM port the device appears on your computer.

1. `idf.py -p com4 erase-flash` 
2. `idf.py menuconfig`
3. `idf.py -p com4 flash`
4. Get the MAC address from console (the other port for ESP32-C6)
5. Create a manufacturing partition using [Zigbee Manufacturing Partition Generator Utility](https://github.com/espressif/esp-zigbee-sdk/blob/main/tools/mfg_tool/README.md).
6. Clone the zigbee sdk repo and remove the `from future.moves.itertools import zip_longest` from `tools/esp_zb_mfg_tool.py`.
7. Install dependencies in ESP-IDF shell: `python3 -m pip install -r requirements.txt` and `python3 -m pip install cryptography esp-idf-nvs-partition-gen crcmod`
8. Generate an installcode with Python: `python3 -c "import os;import crcmod.predefined;installcode=os.urandom(12);crc=crcmod.predefined.mkCrcFun('crc-16');print(installcode.hex() + hex(crc(installcode))[2:]);"`
9. Install dependencies for mgf tool: `python -m pip install esp_idf_nvs_partition_gen future`
10. Run: `python3 esp_zb_mfg_tool.py -i 7f91fbafb9ec53ee8097b3bb8fce -m 404ccafffe5627c4 -c 0x8000 -mn Espressif -mc 0x1338` (change first parameter to random string from previous command, second is MAC without colons)
11. Flash the binary to the `zb_fct` partition (at `0xf5000`): `esptool.py -p com4 write_flash 0xf5000 404ccafffe5627c4.bin`