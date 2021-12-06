
# **Work in progress**

# Introduction
[NEORV32 RISC-V CPU](https://github.com/stnolting/neorv32) implemented on a [Arduino MKR Vidor 4000](https://store.arduino.cc/products/arduino-mkr-vidor-4000)

**Design overview:**
- Currently no support for bitfile transfer by the SAMD21 to the FPGA, bitfile must be programmed by JTAG
- JTAG header/cables should be populated
- Flash and SDRAM enabled
- Current release implements support for OpenOCD

# Compiling the design

First, initialize the submodules (in case a fresh clone)

    git submodule init
    git submodule update

Open the quartus project under syn/neorv32_mkr_vidor_4000.qpf

