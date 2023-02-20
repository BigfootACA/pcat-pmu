obj-m := pcat-pmu.o

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
VERSION ?= $(shell cat VERSION)

default:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install

load:
	-/sbin/rmmod pcat-pmu
	/sbin/insmod pcat-pmu.ko

dkms.conf: dkms.conf.in
	sed "s/@@VERSION@@/$(VERSION)/" $^ > $@

dkms-add: dkms.conf
	/usr/sbin/dkms add $(CURDIR)

dkms-build: dkms.conf
	/usr/sbin/dkms build pcat-pmu/$(VERSION)

dkms-install: dkms.conf
	/usr/sbin/dkms install pcat-pmu/$(VERSION)

dkms-remove: dkms.conf
	/usr/sbin/dkms remove pcat-pmu/$(VERSION) --all

modprobe-install:
	modprobe pcat-pmu

modprobe-remove:
	modprobe -r pcat-pmu

dev: modprobe-remove dkms-remove dkms-add dkms-build dkms-install modprobe-install
