#!/bin/bash

# Check arguments
if [ $# -ne 1 ]
then
    echo "Correct Usage:"
    echo " ./build-pico-pr-static.sh   <project name>"
    exit
fi

proname=$1
# time vivado -log gen.log -journal gen.jou -mode batch -source PicoJITStatic.tcl -tclargs $proname
time vivado -mode batch -source PicoJITStatic.tcl -tclargs $proname
