#!/bin/bash


export PATH=$PATH:../ToolChain/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf/bin
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

#step 1:clean kernel
#make distclean
#make clean 

#step 2:copy uboot config file 
make am335x_evm_nandboot_defconfig


#step 3:compiler uboot
make 

ls -la MLO u-boot.img


