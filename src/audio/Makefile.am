COMP_SRC = \
	eq_iir.c \
	iir.c \
	eq_fir.c \
	fir.c \
	tone.c \
	src.c \
	src_core.c \
	mixer.c \
	mux.c \
	volume.c \
	switch.c \
	dai.c \
	host.c \
	pipeline.c \
	component.c \
	buffer.c

if BUILD_LIB

# only host builds shared libraries, the rest are static
if BUILD_HOST

lib_LTLIBRARIES  = libsof.la

libsof_la_SOURCES = $(COMP_SRC)

libsof_la_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

libsof_la_LDFLAGS = \
	-version-info $(VERSION) \
	-no-undefined \
	-export-dynamic

if HAVE_SSE42
lib_LTLIBRARIES  += libsof_sse42.la

libsof_sse42_la_SOURCES = $(COMP_SRC)

libsof_sse42_la_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(SSE42_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

libsof_sse42_la_LDFLAGS = \
	-version-info $(VERSION) \
	-no-undefined \
	-export-dynamic
endif

if HAVE_AVX
lib_LTLIBRARIES  += libsof_avx.la

libsof_avx_la_SOURCES = $(COMP_SRC)

libsof_avx_la_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(AVX_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

libsof_avx_la_LDFLAGS = \
	-version-info $(VERSION) \
	-no-undefined \
	-export-dynamic
endif

if HAVE_AVX2
lib_LTLIBRARIES  += libsof_avx2.la

libsof_avx2_la_SOURCES = $(COMP_SRC)

libsof_avx2_la_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(AVX2_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

libsof_avx2_la_LDFLAGS = \
	-version-info $(VERSION) \
	-no-undefined \
	-export-dynamic
endif

if HAVE_FMA
lib_LTLIBRARIES  += libsof_fma.la

libsof_fma_la_SOURCES = $(COMP_SRC)

libsof_fma_la_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(FMA_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

libsof_fma_la_LDFLAGS = \
	-version-info $(VERSION) \
	-no-undefined \
	-export-dynamic
endif

else

# Build for non host targets

lib_LIBRARIES  = libsof.a

libsof_a_SOURCES = $(COMP_SRC)

libsof_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

if HAVE_HIFI2EP
lib_LIBRARIES  += libsof_hifi2ep.a

libsof_hifi2ep_a_SOURCES = $(COMP_SRC)

libsof_hifi2ep_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(SSE42_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)
endif

if HAVE_HIFI3
lib_LIBRARIES  += libsof_hifi3.a

libsof_hifi3_a_SOURCES = $(COMP_SRC)

libsof_hifi3_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(SSE42_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)
endif

endif

else

# build for firmware image

noinst_LIBRARIES = libaudio.a

libaudio_a_SOURCES = \
	eq_iir.c \
	iir.c \
	eq_fir.c \
	fir.c \
	tone.c \
	src.c \
	src_core.c \
	mixer.c \
	mux.c \
	volume.c \
	switch.c \
	dai.c \
	host.c \
	pipeline.c \
	pipeline_static.c \
	component.c \
	buffer.c

libaudio_a_CFLAGS = \
	$(ARCH_CFLAGS) \
	$(REEF_INCDIR) \
	$(ARCH_INCDIR) \
	$(PLATFORM_INCDIR)

endif
