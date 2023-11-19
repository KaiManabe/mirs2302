#!/bin/sh
screen -S lidar -X quit
screen -UAmdS lidar /home/mirs2302/git/mirs2302/jetson/ultra_simple --channel --serial /dev/ttyUSB0 1000000