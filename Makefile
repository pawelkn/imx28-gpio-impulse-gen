obj-m	+= imx28-gpio-pwm.o

all:
	make -C $(KERNEL_DIR) SUBDIRS=$(shell pwd) modules
clean:
	make -C $(KERNEL_DIR) SUBDIRS=$(shell pwd) clean
