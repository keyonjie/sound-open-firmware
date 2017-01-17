# build all images for all targets.
pwd=`pwd`

rm -fr src/arch/xtensa/*.ri

# Build for Baytrail
make clean
./configure --with-arch=xtensa --with-platform=baytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf
make
make bin

# Build for Cherrytrail
make clean
./configure --with-arch=xtensa --with-platform=cherrytrail --with-root-dir=$pwd/../xtensa-root/xtensa-byt-elf --host=xtensa-byt-elf
make
make bin

# list all the images
ls -l src/arch/xtensa/*.ri

# build image for host architecture and platform
# TODO: fix this for any host architecture. This should be detected by configure ?
./configure --with-arch=host --with-platform=host --host=x86_64-unknown-linux-gnu --with-root-dir=. --with-hosttarget=baytrail
#./configure --with-arch=host --with-platform=host --host=arm-eabi-linux-gnu --with-root-dir=.
make

