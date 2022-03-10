#!/bin/bash

XML=./configs/demo.xml
VIDEO=./videos/car_640x360.mp4

./build/main -d=10 -si=25 -v=${VIDEO} -xml=${XML}

