#!/bin/bash

set -e

format_code() {
    find $(pwd)/src -iname *.h -o -iname *.cc | xargs clang-format -i  --style=Google
    find $(pwd)/test -iname *.h -o -iname *.cc | xargs clang-format -i  --style=Google
}

if [[ -z  $(command -v clang-format ) ]]; then
    sudo apt-get install clang-format -y
else 
    format_code
fi 

if [[ ! -d $(pwd)/build ]]; then
    cmake -S. -Bbuild
fi

pushd $(pwd)/build > /dev/null;
    cmake ..
    make
popd > /dev/null;