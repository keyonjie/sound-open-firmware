AC_PREREQ([2.69])
AC_INIT([Reef],[m4_esyscmd(./git-version.sh)],[sound-open-firmware@alsa-project.org])
AC_CONFIG_SRCDIR([src/init/init.c])
AC_CONFIG_HEADERS([src/include/config.h])
AC_CONFIG_MACRO_DIRS([m4])
AM_INIT_AUTOMAKE([foreign 1.11 -Wall -Wno-portability subdir-objects silent-rules color-tests dist-xz tar-ustar])

# Initialize maintainer mode
AM_MAINTAINER_MODE([enable])

# get version info from git
m4_define(reef_major, `cat .version | cut -dv -f2 | cut -d. -f1`)
m4_define(reef_minor, `cat .version | cut -d. -f2 | cut -d- -f1`)
AC_DEFINE_UNQUOTED([REEF_MAJOR], reef_major, [Reef major version])
AC_DEFINE_UNQUOTED([REEF_MINOR], reef_minor, [Reef minor version])

AC_CANONICAL_HOST

# General compiler flags
CFLAGS="${CFLAGS:+$CFLAGS } -g -Wall -Werror -Wl,-EL -fno-inline-functions -nostdlib -Wmissing-prototypes"

# Cross compiler tool libgcc and headers
AC_ARG_WITH([root-dir],
        AS_HELP_STRING([--with-root-dir], [Specify location of cross gcc libraries and headers]),
        [], [with_root_dir=no])

# check if we are building FW image or library
AC_ARG_ENABLE(library, [AS_HELP_STRING([--enable-library],[build library])], have_library=$enableval, have_library=no)
if test "$have_library" = "yes"; then
	AC_DEFINE([CONFIG_LIB], [1], [Configure for Shared Library])
fi
AM_CONDITIONAL(BUILD_LIB, test "$have_library" = "yes")

# Architecture support
AC_ARG_WITH([arch],
        AS_HELP_STRING([--with-arch], [Specify DSP architecture]),
	[], [with_arch=no])

case "$with_arch" in
    xtensa*)

	ARCH_CFLAGS="-mtext-section-literals"
	AC_SUBST(ARCH_CFLAGS)

	ARCH_LDFLAGS="-nostdlib -Wl,--no-check-sections -u call_user_start -Wl,-static"
	AC_SUBST(XTENSA_LDFLAGS)

	# extra CFLAGS defined here otherwise configure working gcc tests fails.
	CFLAGS="${CFLAGS:+$CFLAGS }-mlongcalls -O2"
	LDFLAGS="${LDFLAGS:+$LDFLAGS }-nostdlib"

	#ARCH_ASFLAGS=""
	AC_SUBST(ARCH_ASFLAGS)

	ARCH="xtensa"
	AC_SUBST(ARCH)

	AS_IF([test "x$with_root_dir" = xno],
	AC_MSG_ERROR([Please specify cross compiler root header directory]),
		[ROOT_DIR=$with_root_dir])
		AC_SUBST(ROOT_DIR)

    ;;
    host*)

	ARCH_CFLAGS="-g"
	AC_SUBST(ARCH_CFLAGS)

	# extra CFLAGS defined here otherwise configure working gcc tests fails.
	CFLAGS="${CFLAGS:+$CFLAGS } -O3"
	LDFLAGS="${LDFLAGS:+$LDFLAGS }-lpthread"

	ARCH="host"
	AC_SUBST(ARCH)
    ;;
    *)
        AC_MSG_ERROR([Architecture not unknown or not supported. Please specify with --with_arch=])
    ;;
esac

AM_CONDITIONAL(BUILD_XTENSA, test "$ARCH" = "xtensa")
AM_CONDITIONAL(BUILD_HOST, test "$ARCH" = "host")


# Platform support
AC_ARG_WITH([platform],
        AS_HELP_STRING([--with-platform], [Specify Host Platform]),
	[], [with_platform=no])

case "$with_platform" in
    baytrail*)

	PLATFORM_LDSCRIPT="baytrail.x"
	AC_SUBST(PLATFORM_LDSCRIPT)

	PLATFORM="baytrail"
	AC_SUBST(PLATFORM)

	FW_NAME="byt"
	AC_SUBST(FW_NAME)

	XTENSA_CORE="hifi2_std"
	AC_SUBST(XTENSA_CORE)

	have_hifi2ep=yes

	AC_DEFINE([CONFIG_BAYTRAIL], [1], [Configure for Baytrail])
    ;;
    cherrytrail*)

	PLATFORM_LDSCRIPT="baytrail.x"
	AC_SUBST(PLATFORM_LDSCRIPT)

	PLATFORM="baytrail"
	AC_SUBST(PLATFORM)

	FW_NAME="cht"
	AC_SUBST(FW_NAME)

	XTENSA_CORE="hifi2_std"
	AC_SUBST(XTENSA_CORE)

	have_hifi2ep=yes

	AC_DEFINE([CONFIG_CHERRYTRAIL], [1], [Configure for Cherrytrail])
    ;;
    *)
	if test "$ARCH" = "host"; then
		PLATFORM="host"
	else
		AC_MSG_ERROR([Host platform unknown or not supported. Please specify with --with-platform=])
	fi

    ;;
esac

AM_CONDITIONAL(BUILD_BAYTRAIL, test "$FW_NAME" = "byt")
AM_CONDITIONAL(BUILD_CHERRYTRAIL,  test "$FW_NAME" = "cht")
AM_CONDITIONAL(BUILD_HASWELL,  test "$FW_NAME" = "hsw")
AM_CONDITIONAL(BUILD_BROADWELL,  test "$FW_NAME" = "bdw")
AM_CONDITIONAL(BUILD_APOLLOLAKE,  test "$FW_NAME" = "apl")

# DSP core support (Optional)
AC_ARG_WITH([dsp-core],
        AS_HELP_STRING([--with-dsp-core], [Specify DSP Core]),
	[], [with_dsp_core=no])

case "$with_dsp_core" in
    CHT_audio_hifiep*)
	# BXT/CHT DSP Core
	XTENSA_CORE="CHT_audio_hifiep"
	AC_SUBST(XTENSA_CORE)
    ;;

esac

# Optimisation settings and checks

# SSE4_2 support
AC_ARG_ENABLE(sse42, [AS_HELP_STRING([--enable-sse42],[enable SSE42 optimizations])], have_sse42=$enableval, have_sse42=yes)
AX_CHECK_COMPILE_FLAG(-msse4.2, [SSE42_CFLAGS="-DOPS_SSE42 -msse4.2 -ffast-math -ftree-vectorizer-verbose=0"],
	[have_sse42=no])
if test "$have_sse42" = "yes"; then
	AC_DEFINE(HAVE_SSE42,1,[Define to enable SSE42 optimizations.])
fi
AM_CONDITIONAL(HAVE_SSE42, test "$have_sse42" = "yes")
AC_SUBST(SSE42_CFLAGS)

# AVX support
AC_ARG_ENABLE(avx, [AS_HELP_STRING([--enable-avx],[enable AVX optimizations])], have_avx=$enableval, have_avx=yes)
AX_CHECK_COMPILE_FLAG(-mavx, [AVX_CFLAGS="-DOPS_AVX -mavx -ffast-math -ftree-vectorizer-verbose=0"],
	[have_avx=no])
if test "$have_avx" = "yes"; then
	AC_DEFINE(HAVE_AVX,1,[Define to enable AVX optimizations.])
fi
AM_CONDITIONAL(HAVE_AVX, test "$have_avx" = "yes")
AC_SUBST(AVX_CFLAGS)


# AVX2 support
AC_ARG_ENABLE(avx2, [AS_HELP_STRING([--enable-avx2],[enable AVX2 optimizations])], have_avx2=$enableval, have_avx2=yes)
AX_CHECK_COMPILE_FLAG(-mavx2, [AVX2_CFLAGS="-DOPS_AVX2 -mavx2 -ffast-math -ftree-vectorizer-verbose=0"],
		[have_avx2=no])
if test "$have_avx2" = "yes"; then
	AC_DEFINE(HAVE_AVX2,1,[Define to enable AVX2 optimizations.])
fi
AM_CONDITIONAL(HAVE_AVX2, test "$have_avx2" = "yes")
AC_SUBST(AVX2_CFLAGS)


# FMA support
AC_ARG_ENABLE(fma, [AS_HELP_STRING([--enable-fma],[enable FMA optimizations])], have_fma=$enableval, have_fma=yes)
AX_CHECK_COMPILE_FLAG(-mfma, [FMA_CFLAGS="-DOPS_FMA -mfma -ffast-math -ftree-vectorizer-verbose=0"],
	[have_fma=no])
if test "$have_fma" = "yes"; then
	AC_DEFINE(HAVE_FMA,1,[Define to enable FMA optimizations.])
fi
AM_CONDITIONAL(HAVE_FMA, test "$have_fma" = "yes")
AC_SUBST(FMA_CFLAGS)

# Hifi2EP
AC_ARG_ENABLE(hifi2ep, [AS_HELP_STRING([--enable-hifi2ep],[enable HiFi2EP optimizations])], have_hifi2ep=$enableval, have_hifi2ep=yes)
AX_CHECK_COMPILE_FLAG(-mhifi2ep, [FMA_CFLAGS="-DOPS_HIFI2EP -mhifi2ep -ffast-math -ftree-vectorizer-verbose=0"],
	[have_hifi2ep=no])
if test "$have_hifi2ep" = "yes"; then
	AC_DEFINE(HAVE_HIFI2EP,1,[Define to enable Hifi2 EP optimizations.])
fi
AM_CONDITIONAL(HAVE_HIFI2EP, test "$have_hifi2ep" = "yes")
AC_SUBST(HIFI2EP_CFLAGS)

# Hifi3
AC_ARG_ENABLE(hifi3, [AS_HELP_STRING([--enable-hifi3],[enable HiFi3 optimizations])], have_hifi3=$enableval, have_hifi3=yes)
AX_CHECK_COMPILE_FLAG(-mhihi3, [FMA_CFLAGS="-DOPS_HIFI3 -mhifi3 -ffast-math -ftree-vectorizer-verbose=0"],
	[have_hifi3=no])
if test "$have_hifi3" = "yes"; then
	AC_DEFINE(HAVE_HIFI3,1,[Define to enable Hifi3 optimizations.])
fi
AM_CONDITIONAL(HAVE_HIFI3, test "$have_hifi3" = "yes")
AC_SUBST(HIFI3_CFLAGS)


# Test after CFLAGS set othewise test of cross compiler fails. 
AM_PROG_AS
AM_PROG_AR
AC_PROG_CC
LT_INIT
AC_CHECK_TOOL([OBJCOPY], [objcopy], [])
AC_CHECK_TOOL([OBJDUMP], [objdump], [])

AM_EXTRA_RECURSIVE_TARGETS([bin])

AM_EXTRA_RECURSIVE_TARGETS([vminstall])

AC_CONFIG_FILES([
	Makefile
	src/Makefile
	src/tasks/Makefile
	src/init/Makefile
	src/arch/Makefile
	src/arch/xtensa/Makefile
	src/arch/xtensa/include/Makefile
	src/arch/xtensa/hal/Makefile
	src/arch/xtensa/xtos/Makefile
	src/arch/host/Makefile
	src/arch/host/include/Makefile
	src/audio/Makefile
        src/math/Makefile
	src/drivers/Makefile
	src/include/Makefile
	src/include/reef/Makefile
	src/include/reef/audio/Makefile
	src/include/uapi/Makefile
	src/ipc/Makefile
	src/lib/Makefile
	src/platform/Makefile
	src/platform/baytrail/Makefile
	src/platform/baytrail/include/Makefile
	src/platform/baytrail/include/platform/Makefile
	src/platform/baytrail/include/xtensa/Makefile
	src/platform/baytrail/include/xtensa/config/Makefile
	src/library/Makefile
	src/library/include/Makefile
	src/library/include/platform/Makefile
])
AC_OUTPUT

AS_IF([test "$have_sse42" = "yes"], ENABLE_SSE42=yes, ENABLE_SSE42=no)
AS_IF([test "$have_avx" = "yes"], ENABLE_AVX=yes, ENABLE_AVX=no)
AS_IF([test "$have_avx2" = "yes"], ENABLE_AVX2=yes, ENABLE_AVX2=no)
AS_IF([test "$have_fma" = "yes"], ENABLE_FMA=yes, ENABLE_FMA=no)
AS_IF([test "$have_hifi2ep" = "yes"], ENABLE_HIFI2EP=yes, ENABLE_HIFI2EP=no)
AS_IF([test "$have_hifi3" = "yes"], ENABLE_HIFI3=yes, ENABLE_HIFI3=no)

AS_IF([test "$have_library" = "yes"], ENABLE_LIBRARY=yes, ENABLE_LIBRARY=no)

echo "
---{ $PACKAGE_NAME $VERSION }---

Target Architecture:           ${ARCH}
Target Platform:               ${PLATFORM}
Target Core:                   ${XTENSA_CORE}
Build Library:                 ${ENABLE_LIBRARY}

Compiler:                      ${CC}
CFLAGS:                        ${CFLAGS}
LDFLAGS:                       ${LDFLAGS}
ARCH_CFLAGS:                   ${ARCH_CFLAGS}
ARCH_LDFLAGS:                  ${ARCH_LDFLAGS}

Target Specific Optimisations

Enable SSE42:                  ${ENABLE_SSE42}
Enable AVX:                    ${ENABLE_AVX}
Enable AVX2:                   ${ENABLE_AVX2}
Enable FMA:                    ${ENABLE_FMA}
Enable HIFI2EP:                ${ENABLE_HIFI2EP}
Enable HIFI3:                  ${ENABLE_HIFI3}
"

