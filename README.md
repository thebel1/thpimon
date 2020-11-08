# thpimon
Native ESXi on Arm hardware status driver for the Raspberry Pi.

Using the Python library, you can currently get the board revision and temperature.

Currently work-in-progress, the driver is capable of communicating with the VideoCore firmware using fixed-size buffers (so no console).

### Installation

To install the driver, download the VIB file in the build directory and copy it to your RPi.

Install it as described here: https://kb.vmware.com/s/article/2008939

I would recommend downloading the Python library in ./pyUtil/pimonLib/ to interact with the thpimon character device.

After the VIB has been installed:

1. Reboot the RPi
2. Run the setup script located at /path/to/pyUtil/pimonLib/setup.sh
3. Take a screenshot using: `/path/to/pyUtil/screenshot.py`
4. SCP `/tmp/screenshot.bmp` to your local machine and open it