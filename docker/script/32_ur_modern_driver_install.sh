#!/bin/bash

DEPS_DIR=${DEPS_PATH}
SUDO=$1
if [ "$SUDO" == "sudo" ];then
        SUDO="sudo"
else
        SUDO=""
fi

# install ur_modern_driver
cd $DEPS_DIR
git clone --depth 1 https://github.com/st99315/ur_modern_driver.git -b kinetic-devel
cd ur_modern_driver/libur_modern_driver
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make
$SUDO make install
