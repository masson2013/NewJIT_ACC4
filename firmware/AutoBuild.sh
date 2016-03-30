#!/bin/bash
#Sen Ma
build-pico-fw.sh m505lx325.fwproj project
./build-pico-jit-static.sh M505_LX325T_NewJIT_ACC4
./build-pico-jit-module.sh M505_LX325T_NewJIT_ACC4 acc_vadd.dcp
./build-pico-jit-module.sh M505_LX325T_NewJIT_ACC4 acc_vmul.dcp
./build-pico-jit-module.sh M505_LX325T_NewJIT_ACC4 acc_vredu.dcp
