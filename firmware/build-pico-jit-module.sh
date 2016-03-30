#!/bin/bash

# Check arguments
if [ $# -ne 2 ]
then
    echo "Correct Usage:"
    echo " ./build-pico-pr-module.sh   <project name> <module name>"
    exit
fi

proname=$1
modulename=$2
time vivado -log gen.log -journal gen.jou -mode batch -source PicoJITModule.tcl -tclargs $proname $modulename
# python ../software/bit_h_gen.py
