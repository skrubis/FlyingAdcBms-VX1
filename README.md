# Flying ADC BMS
This BMS senses up to 16 channels per board. Boards can be stacked to sense any number of cells. Each board has a current sensor input therefor the current sensor can be located where it is most convenient. In addition each board has two temperature sensor inputs.

For cell voltage sensing a single channel, high precision delta/sigma ADC is used in conjunction with a 16 channel multiplexer. Each cell is sensed with a differential input with an accuracy 0.5 mV. The ADC is paired with a full bridge connected to its 5V isolated power supply. This allows sending current into a low cell and also draining current from a high cell. Only small balancing currents are used over a long time thus omitting large power resistors.
Due to its architecture the cell inputs are somewhat tolerant to miswiring, i.e. a swapped cell lead or multiple cells across one input won't damage the circuitry and will instead be reported by software.

# Wiring
Each board has 17 inputs for cell taps labeled V0 to V16. The very first input of the first board is connected to B-, i.e. the negative pole of the most negative cell. V16 of the first board is connected to the positive pole of the 16th cell. The latter is also connected to V0 of the second board.
Not all channels of a board need to be used if it is more convenient

A common CAN bus and 12V supply is connected to each module. In addition each module has an enable input an output. The first enable input must be connected to 12V whenever you want the BMS to run. subsequent enable inputs are connected to the enable output of the previous module.

Further documentation can be found here: https://openinverter.org/wiki/16-cell_BMS

# OTA (over the air upgrade)
The firmware is linked to leave the 4 kb of flash unused. Those 4 kb are reserved for the bootloader
that you can find here: https://github.com/jsphuebner/stm32-CANBootloader/

When flashing your device for the first time you must first flash that bootloader. After that you can
use the ESP32 CAN module and its web interface to upload your actual application firmware.

The web interface is here: https://github.com/jsphuebner/esp32-web-interface/tree/can-backend

Binaries are here: https://github.com/jsphuebner/FlyingAdcBms/actions

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Ubuntu type

`sudo apt-get install git gcc-arm-none-eabi`

The only external depedencies are libopencm3 and libopeninv. You can download and build these dependencies by typing

`make get-deps`

Now you can compile stm32-<yourname> by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface.

# Editing
The repository provides a project file for Code::Blocks, a rather leightweight IDE for cpp code editing.
For building though, it just executes the above command. Its build system is not actually used.
Consequently you can use your favority IDE or editor for editing files.

# Configuration parameters (advanced)
The firmware exposes configuration parameters via the parameter system (see `include/param_prj.h`).
Below are advanced parameters related to balancing focus mode, target computation, and older hardware timing.

- balFocusEnable (0/1, default 1)
  Enables the Focused Weak-Cell mode. When enabled, the BMS can hold on the weakest cell(s) for long durations to accelerate recovery.

- balFocusMinDev (mV, default 15)
  Minimum deviation below the balance target required for a cell to qualify for the focus set.

- balFocusTopN (1..4, default 1)
  Number of cells to include in the focus set (typically 1). The most deviating cell becomes the active focus channel.

- balFocusHold (s, default 600)
  Hold time per focused cell. During this time, the channel is pinned to the focused cell to maximize balancing duty.

- balAllowRun (0/1, default 1)
  Allows balancing while in RUN if |idcavg| is below the `idlecurrent` threshold, effectively treating low-current RUN like IDLE.

- balTargetTrimLowK (cells, default 1)
  Trims the balance target by excluding the K lowest cells when computing the target. This prevents one weak cell from pulling the target down and causing unnecessary discharge of already-balanced cells. Note: this affects the target used for balancing decisions only; the displayed `uavg/umin/umax` remain untrimmed.
  Set to 0 to revert to legacy behavior.

- balFocusRefresh (s, default 60)
  While focusing a weak cell, perform a full periodic sweep every N seconds to refresh `uavg/umin/umax` and re-evaluate the focus set.

- muxSettleExtra (ms, default 6; effective on older HW only)
  Adds extra settle time between MUX channel select and ADC start to prevent repeated identical readings on older boards. Applied only on HW_20/HW_21/HW_22/HW_24; ignored on HW_23 and HWV1.

Notes:
- When focus mode is active, the focused cell is never discharged (charge-only on that channel), and the `ubalance` gate is bypassed so weak cells can be raised even when average voltage is low.
- The trimmed target affects balancing decisions (charge/discharge thresholding) but does not alter the reported `uavg/umin/umax` values.

# Adding classes or modules
As your firmware grows you probably want to add classes. To do so, put the header file in include/ and the 
source file in src/ . Then add your module to the object list in Makefile that starts in line 43 with .o
extension. So if your files are called "mymodule.cpp" and "mymodule.h" you add "mymodule.o" to the list.

When changing a header file the build system doesn't always detect this, so you have to "make clean" and
then make. This is especially important when editing the "*_prj.h" files.
