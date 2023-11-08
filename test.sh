#!/bin/bash


DEVICE_NAME=/dev/video1

start_with_gdb() {
gdb  --args ./build/cam_node --type native \
                 --cam_name ud_camera1 \
                 --dev_name $DEVICE_NAME \
                 --width 1920 \
                 --height 1080 \
                 --io_method 2 \ #dma
}

normal_start() {
./build/cam_node --type native \
                 --cam_name ud_camera1 \
                 --dev_name $DEVICE_NAME \
                 --width 1920 \
                 --height 1080 \
                 --io_method 2 \ #dma
}


if [[ $# -gt 0 ]]; then
	 case $1 in
		-g)
			echo "start with gdb..."
			start_with_gdb
			;;
		-n)
			echo "normal start..."
			normal_start
			;;
	 esac
fi
