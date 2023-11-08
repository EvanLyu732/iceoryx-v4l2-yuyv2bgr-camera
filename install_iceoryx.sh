#!/bin/bash

########################## 
# Install Iceoryx        #
##########################

sudo apt install -y gcc g++ cmake libacl1-dev libncurses5-dev pkg-config

cd /tmp
git clone https://github.com/eclipse-iceoryx/iceoryx.git -b v2.0.3
cd iceoryx
cmake -Bbuild -Hiceoryx_meta
cmake -Bbuild -Hiceoryx_meta -DCMAKE_PREFIX_PATH=$(PWD)/build/dependencies/
cmake --build build

sudo cmake --build build --target install