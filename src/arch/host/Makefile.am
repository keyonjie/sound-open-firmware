SUBDIRS = include

noinst_LIBRARIES = \
	libhost.a

libhost_a_SOURCES = \
	init.c

libhost_a_CFLAGS = \
	$(ARCH_INCDIR) \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(PLATFORM_INCDIR)

bin_PROGRAMS = \
	reef

reef_SOURCES = \
	main.c

reef_CFLAGS = \
	$(ARCH_INCDIR) \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(PLATFORM_INCDIR)

reef_CCASFLAGS = \
	$(ARCH_INCDIR) \
	$(ARCH_ASFLAGS) \
	$(PLATFORM_INCDIR)

reef_LDADD = \
	../../init/libinit.a \
	libhost.a \
	../../platform/$(PLATFORM)/libplatform.a \
	../../tasks/libtasks.a \
	../../audio/libaudio.a \
	../../lib/libcore.a \
	../../ipc/libipc.a \
	../../drivers/libdrivers.a \
	../../platform/$(PLATFORM)/libplatform.a \
	../../drivers/libdrivers.a \
	-lpthread \
	-lc

reef_LDFLAGS = \
	$(ARCH_LDFLAGS)

bin-local: reef
	$(OBJCOPY) -O binary reef reef.bin
	$(OBJDUMP) -D reef > reef-$(FW_NAME).map
	rimage -i reef -o reef-$(FW_NAME).ri -m $(FW_NAME)

