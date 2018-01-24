noinst_LIBRARIES = libdrivers.a

libdrivers_a_SOURCES = \
	ssp.c \
	dw-dma.c

if BUILD_APOLLOLAKE
libdrivers_a_SOURCES += \
	hda-dma.c
endif

if BUILD_CANNONLAKE
libdrivers_a_SOURCES += \
	hda-dma.c
endif

libdrivers_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)
