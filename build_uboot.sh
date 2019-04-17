#!/bin/bash
export CPUS=`grep -c processor /proc/cpuinfo`
export ARCH=arm
export CROSS_COMPILE=/home/book/ti-processor-sdk-linux-am335x-evm-03.03.00.04/linux-devkit/sysroots/x86_64-arago-linux/usr/bin/arm-linux-gnueabihf-
export          PATH=/home/book/ti-processor-sdk-linux-am335x-evm-03.03.00.04/linux-devkit/sysroots/x86_64-arago-linux/usr/bin:$PATH



#step 1:clean kernel
#make distclean
#make clean 

#step 2:copy uboot config file 
make 100ask_am335x_v1_defconfig


#step 3:compiler uboot
make 


