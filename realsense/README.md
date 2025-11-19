# Realsense Meta-package

## Installation

run install script:
```bash
./scripts/install_realsense_d435.sh
```

### Jetson
- follow the official instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)

## Troubleshooting

### libusb

Bluefox driver comes with libusb that is missing some important functions. As a result, realsense won't be able to launch if bluefox camera driver is installed.

Solution:

Remove the conflicting libusb acquired during bluefox installation.
```bash
rm /opt/mvIMPACT_acquire/lib/x86_64/libusb-1.0.so.0.0.0
```
### udev rules reloading problem

Check if you have correct prime-select, if you haven't connected the doc station it should be intel ( depends on your system ), to check use
```bash
sudo prime-select query
```
To select use
```bash
sudo prime-select intel
```

### Realsense enumerated as USB 2.0 device

Plugging the USB 3.0 original cable slowly into the USB port of the host causes it to be enumerated as USB 2.0 device. The current USB mode can be checked by `lsusb -t` - 5000M is USB 3.0, 480M is USB 2.0. Enumerating Realsense D435 as USB 2.0 devices causes some image streams to be unavailable.

Solution:

Insert the cable quickly to the USB port.
