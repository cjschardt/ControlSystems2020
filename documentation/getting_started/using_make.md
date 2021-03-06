# How to use make

## Make Command Syntax

    make <TARGET> <VARIABLE_1>=<VALUE_1> <VARIABLE_n>=<VALUE_n>

`make` is the command that you need to type initially to run the make script. If
you execute it alone at the top of the SJSU-Dev2 folder it will dump a help menu
of information. Example:

    make

### Target

`TARGET` is the thing you want build or an action you want to take. An example
targets that build stuff would be `application`, `test`, and `flash`. The
`application` target would make an application binary, `make test` will build
unit test code and run the test executable on your machine.

And example of making an application:

    make application

### Variables

`VARIABLE`: name of a variable within the project that you can change to modify
how the target is built. The most used variable is the `PLATFORM` variable which
you use to change which platform you are building the code for.

`VALUE`: simply the value you want to put in the variable. The set of values you
can put in the variable depends greatly on the actual variable. For example, the
possible set of VALUES for VARIABLE `PLATFORM` is the set of microcontrollers we
support. Example:

    make application PLATFORM=lpc40xx

## USAGE

    make [target] [PLATFORM=[linux|lpc40xx|lpc17xx|...]]
    make [target] [PLATFORM=platform] [JTAG=[stlink|jlink|...]]
    make [target] [PLATFORM=platform] OPTIMIZE=2


## TARGETS

### GENERAL TARGETS
#### `help`
Shows this menu.

#### `application`
Compiles code and builds application binary.

#### `execute`
#### `flash`
Program applications binary to device or executes binary on linux.
Only supported for LPC series devices such as lpc17xx and lpc40xx.
This option will also build the binary, if necessary, before programming or
executing the binary.

If you have a JTAG or SWD debugging device and can access the jtag/swd pins
of the device, it is preferred to use that as programming is orders
magnitude faster.

#### `program`
Program application binary

#### `openocd`
Opens openocd and connects to the selected PLATFORM via the selected JTAG
device.

#### `debug`
Will bring up GDB with current projects .elf file, using openocd if
necessary.

#### `clean`
Deletes temporary build files found within the build folder of the project.
Keeping Temporary build files speeds up builds, so keeping them around is
usually a good thing.
Clean will also delete application binaries, disassemblies, test files and
anything else found in the project's "build" folder.

#### `library-clean`
Cleans temporary libraries files made specifically for the designated
platform.

#### `purge`
Remove all temporary local build files and static libraries

#### `telemetry`
Launch telemetry web interface on platform

### DEVELOPER TARGETS

#### `lint`
Check that source files abide by the SJSU-Dev2 coding standard.

#### `tidy`
Check that source file fit the SJSU-Dev2 naming convention.

#### `test`
Build all tests as defined in USER_TESTS, typically defined within the
project.mk file.

#### `library-test`
Compile and test SJSU-Dev2's library test suite.

#### `presubmit`
Runs the presubmit checks which is used for continuous integration. The
following checks will be performed:
  1. Checks that all projects compile without warnings or errors.
  2. Performs `make lint`
  3. Performs `make tidy`
  4. Performs `make library-test`

#### `format-code`
Will format all modified files in commit to the SJSU-Dev2 standard.

#### `debug-test`
Allows you to debug your test executable in GDB.

#### `stacktrace-application`
Usage:
  make stacktrace-application TRACES="0x80000000 0x800001FC0 ..."

Description:
  Will show the stack trace based on the program counter trace addresses in
  the TRACES variable.

### MAKEFILE DEBUG TARGETS

#### `show-variables`
Displays the contents of make variables.


### COMMAND LINE OPTIONS:

#### `PLATFORM`
    Usage:
      ... PLATFORM=lpc40xx
      ... PLATFORM=stm32f10x
      ... PLATFORM=linux

    Description:
      Set the target platform for the make target. For example, if you would
      like to build the "hello_world" project for linux you can run:

        make application PLATFORM=linux

#### `JTAG`
    Usage:
      ... JTAG=stlink
      ... JTAG=jlink
      ... JTAG=ft2232h

    Description:
      Set the JTAG or SWD device to be used for programming or debugging device.
      Some of such debugging devices may be the following:

        stlink, jlink, FT2232H

      The list of all supported debug devices can be found here:
      "tools/openocd/scripts/interface/"

      and this link:

      http://openocd.org/doc/html/Debug-Adapter-Hardware.html#Debug-Adapter-Hardware

#### `OPTIMIZE`
    Usage:
      ... OPTIMIZE=0
      ... OPTIMIZE=2
      ... OPTIMIZE=s

    Description:
      Set the optimization level of the compiler. Default is optimization level
      is 0. Available optimization levels for GCC are:

        0: Low to no optimization
            Only trivial and quick optimizations will be considered.

        1: Level 1 optimization
            Optimizes the executable, reduces binary size, increases execution
            performance, but compilation time will increase.

        2: Level 2 optimization
            Optimizes the executable further. Performs all optimizations that
            do not sacrifice memory to increase runtime performance.

        3: Highest level of optimization
            Best runtime performance, but will typically increase binary size
            above O2.

        s: Optimize for size
            Will perform all optimizations that reduce the size of the binary.
            May sacrifice runtime performance in order to decrease binary size.

#### `DEVICE`
    Usage:
      make flash DEVICE=/dev/ttyUSB0

    Description:
      Set the serial port to use when performing "make flash".

#### `TEST_ARGS`
    Usage:
      make test TEST_ARGS="-s [i2c,adc]"
      make library-test TEST_ARGS="-s [i2c,adc]"
      make library-test TEST_ARGS="-h"

    Description:
      Defines the arguments to be passed to the test executable.
      -h will return the test executable help menu.

#### `COMMON_FLAGS`
    Usage:
      make application COMMON_FLAGS="-Wall"

    Description:
      Add additional compiler flags.

#### `OPENOCD_CONFIG`
    Usage:
      ... OPENOCD_CONFIG=custom_debug_config.cfg

    Description:
      Used to use your own custom debug config script.

#### `LINKER`
    Usage:
      ... LINKER=custom_linker_script.ld

    Description:
      Used to use your own custom linker script.
