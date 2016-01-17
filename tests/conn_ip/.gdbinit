target remote localhost:2331
monitor interface jtag
monitor speed 5000
monitor endian little
monitor flash download = 1
monitor flash breakpoints = 1
monitor reset
