# Udev rules

Udev is a device manager for linux. We use it to trigger an event when a new device is plugged in. In our case, we use it to always give the same name to serial devices whose names are hardcoded somewhere else in the code.

## Example
```console
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps"
```
## Usage

1. Copy and paste the .rules files in  /etc/udev/rules.d/
2. Reload and apply the rules using:
```console
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
