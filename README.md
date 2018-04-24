# SDCAN-LKM
Software Defined CAN - Loadable [Linux] Kernel Module

# Installation

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