LINUX_SRC=/lib/modules/$(shell uname -r)/build

obj-m += adafruit32x16.o

all:
	@$(MAKE) -C $(LINUX_SRC) M=$(PWD) modules

clean:
	@$(MAKE) -C $(LINUX_SRC) M=$(PWD) clean
