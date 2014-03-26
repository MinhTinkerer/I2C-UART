# A simple Makefile to build modules
# if a module called module.ko is genrated from two source files the correct
# incantation would be:
# obj-m      :=module.o
# module-objs:=file1.o file2.o
#
#CC = $(CROSS_COMPILE)gcc
#
#obj-m:=own.o
#
#KERNELDIR ?=/lib/modules/$(shell uname -r)/build
#KERNELDIR :=/home/bamboo/work/hi3511/Kernel/linux-2.6.14
#
#PWD       :=$(shell pwd)
#
#all:
#	make -C /root/Desktop/i2c-urat/new-gpio-485/tests/linux-2.6.37-lcd-buzzer-touch-logo-20-feb M=#$(PWD) modules	
#	make -C /root/Desktop/i2c-urat/new-gpio-485/tests/linux-03.20.00.14 M=$(PWD) modules
#        $(MAKE) -C $(KERNELDIR) M=$(PWD) modules
#        cp -f xr20m1172-485.ko  /home/bamboo/work/hi3511/rootbox
#        
#install:
#        install -d $(INSTALLDIR) 
#        install -c $(TARGET).O $(INSTALLDIR)
#
#clean:
#	make -C /root/Desktop/i2c-urat/new-gpio-485/tests/linux-2.6.37-lcd-buzzer-touch-logo-20-feb M=#$(PWD) clean
#		make -C /root/Desktop/i2c-urat/new-gpio-485/tests/linux-03.20.00.14 M=$(PWD) clean
#        rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .temp_versions *.order \
#               *.markers *symvers

# LED Core
obj-$(CONFIG_I2C_UART_SUPPORT)			+= own.o


