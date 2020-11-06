# thpimon
Native ESXi on Arm hardware status driver for the Raspberry Pi.

Using the Python library, you can currently get the board revision and temperature.

Currently work-in-progress, the driver is capable of communicating with the VideoCore firmware using fixed-size buffers (so no console).

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

Note: you will need to reboot the RPi after installing the VIB.

I would recommend downloading the Python library in ./pyUtil/pimonLib/ to interact with the thpimon character device.
