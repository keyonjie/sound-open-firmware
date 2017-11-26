# fail on any errors
set -e

# run autogen.sh
./autogen.sh

# build all images for all targets.
pwd=`pwd`

rm -fr src/arch/xtensa/*.ri

# build library for host platform architecture
./configure --with-arch=host --enable-library=yes --host=x86_64-unknown-linux-gnu --prefix=$pwd/../host-root/
make
make install

# Build for Baytrail
./configure --with-arch=xtensa --with-platform=baytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf
make clean
make
make bin

# build library for xtensa byt architectures
./configure --with-arch=xtensa --with-platform=baytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf --enable-library=yes --prefix=$pwd/../xtensa-root/xtensa-byt-elf
make clean
make
make install

# Build for Cherrytrail
./configure --with-arch=xtensa --with-platform=cherrytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf
make clean
make
make bin

# build library for xtensa cht architectures
./configure --with-arch=xtensa --with-platform=baytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf --enable-library=yes --prefix=$pwd/../xtensa-root/xtensa-byt-elf
make clean
make
make install

# list all the images
ls -l src/arch/xtensa/*.ri
