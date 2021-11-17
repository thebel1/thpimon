# Note: all the source code contained in this repository is for educational use only, and is provided with no support, endorsement, warranty, guarantee or implied fitness for a particular purpose. It does not constitute sample driver code. Do not reach out to VMware to support this code. Do not copy and reuse in any commercial or product setting.

# thpimon
Native ESXi on Arm hardware status driver for the Raspberry Pi.

Using the Python library, you can currently get the board revision and temperature.

Currently work-in-progress, the driver is capable of communicating with the VideoCore firmware using fixed-size buffers (so no console).

### Sample Output

Taken from a Raspberry Pi 4B:

```
$ /scratch/downloads/pyUtil/pimon_util.py
Firmware Revision:      0x5f440c10
Board Model:            0
Board Revision:         0xc03111
Board MAC Address:      d6:51:4:32:a6:dc
Board Serial:           0x000000664b0067
Temp:                   60.0 (deg. C)
```

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

Note: you will need to reboot the RPi after installing the VIB.

I would recommend downloading the Python library in ./pyUtil/pimonLib/ to interact with the thpimon character device.

Note: You may need to manually change the `PIMON_DEVICE_PATH` variable in `./pyUtil/gpioLib/__init__.py` to reflect the correct name for the character device. I'm digging through the VMKAPI to find a way to name the character device upon creation, so stay tuned.

### Other Branches

If you want to try auto-configuring the name of the `PIMON_DEVICE_PATH`, check out: https://github.com/thebel1/thpimon/tree/autoconf

If you want to take screenshots of your Pi, check out: https://github.com/thebel1/thpimon/tree/fbuf
