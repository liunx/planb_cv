#!/bin/bash

XML=./configs/demo.xml
VIDEO=./videos/test.mp4

./build/main -d=10 --si=25 -v=${VIDEO} -xml=${XML}

