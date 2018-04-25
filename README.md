# SDCAN-LKM
Software Defined CAN - Loadable [Linux] Kernel Module for Raspberry Pi

# Installation

* sudo apt-get update
* sudo apt-get upgrade
* sudo apt-get install raspberrypi-kernel-headers
* sudo apt-get install git
* cd /home/pi
* git clone https://github.com/macaba/SDCAN-LKM.git
* cd SDCAN-LKM/src
* make
* sudo insmod helloWorld.ko
* lsmod
* modinfo helloWorld.ko
* tail -f kern.log
* sudo rmmod helloWorld.ko (to remove)

* dtc -@ -I dts -O dtb -o sdcan-can0.dtbo sdcan-can0-overlay.dts
* sudo cp sdcan-can0.dtbo /boot/overlays
* dtoverlay=sdcan-can0,oscillator=16000000,interrupt=25
* sudo cp SDCAN.ko /lib/modules/`uname -r`/kernel/drivers/net/can/spi
* sudo depmod -a
* sudo modprobe SDCAN
* sudo vim /etc/modules
* ls /sys/bus/spi/devices/spi0.0/net/can0/
* sudo /sbin/ip link set can0 up type can bitrate 500000

# Useful

* sudo raspi-config (to enable SPI)
* apt install hdparm
* curl http://www.nmacleod.com/public/sdbench.sh | sudo bash