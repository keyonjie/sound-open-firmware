noinst_LIBRARIES = libdrivers.a

if BUILD_HOST
libdrivers_a_SOURCES = \
	host-vssp.c \
	host-vdma.c
else
libdrivers_a_SOURCES = \
	ssp.c \
	dw-dma.c
endif

libdrivers_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)