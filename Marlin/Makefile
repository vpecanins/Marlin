# Marlin Firmware Arduino Project Makefile
#
# Makefile Based on:
# Arduino 0011 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
# Marlin adaption by Daid
# Marlin 2.0 support and RELOC_WORKAROUND by @marcio-ao
#
# This has been tested with Arduino 0022.
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# Detailed instructions for using the makefile:
#
#  1. Modify the line containing "ARDUINO_INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under Mac OS X, this
#     might be /Applications/Arduino.app/Contents/Resources/Java).
#
#  2. Modify the line containing "UPLOAD_PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. UPLOAD_PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wild card (e.g. UPLOAD_PORT = /dev/tty.usb*).
#
#  3. Set the line containing "MCU" to match your board's processor.
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000. If you are using Gen7 electronics, you
#     probably need to use 20000000. Either way, you must regenerate
#     the speed lookup table with create_speed_lookuptable.py.
#
#  4. Type "make" and press enter to compile/verify your program.
#
#  5. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# Note that all settings at the top of this file can be overriden from
# the command line with, for example, "make HARDWARE_MOTHERBOARD=71"
#
# To compile for RAMPS (atmega2560) with Arduino 1.6.9 at root/arduino you would use...
#
#   make ARDUINO_VERSION=10609 AVR_TOOLS_PATH=/root/arduino/hardware/tools/avr/bin/ \
#   HARDWARE_MOTHERBOARD=33 ARDUINO_INSTALL_DIR=/root/arduino
#
# To compile and upload simply add "upload" to the end of the line...
#
#   make ARDUINO_VERSION=10609 AVR_TOOLS_PATH=/root/arduino/hardware/tools/avr/bin/ \
#   HARDWARE_MOTHERBOARD=33 ARDUINO_INSTALL_DIR=/root/arduino upload
#
# If uploading doesn't work try adding the parameter "AVRDUDE_PROGRAMMER=wiring" or
# start upload manually (using stk500) like so:
#
#   avrdude -C /root/arduino/hardware/tools/avr/etc/avrdude.conf -v -p m2560 -c stk500 \
#   -U flash:w:applet/Marlin.hex:i -P /dev/ttyUSB0
#
# Or, try disconnecting USB to power down and then reconnecting before running avrdude.
#

# This defines the board to compile for (see boards.h for your board's ID)
# Anet 1.0 (Melzi clone)

HARDWARE_MOTHERBOARD ?= 1509
HARDWARE_VARIANT ?= Sanguino
MCU              ?= atmega1284p
V ?= 1

# Arduino source install directory, and version number
# On most linuxes this will be /usr/share/arduino
ARDUINO_INSTALL_DIR  ?= ${HOME}/3D/arduino-1.8.10
ARDUINO_VERSION      ?= 10810

# The installed Libraries are in the User folder
ARDUINO_USER_DIR ?= ${HOME}/3D/arduino-1.8.10

# You can optionally set a path to the avr-gcc tools. Requires a trailing slash. (ex: /usr/local/avr-gcc/bin)
AVR_TOOLS_PATH ?= ${HOME}/3D/arduino-1.8.10/hardware/tools/avr/bin/

#Programmer configuration
UPLOAD_RATE        ?= 115200
AVRDUDE_PROGRAMMER ?= arduino
# on most linuxes this will be /dev/ttyACM0 or /dev/ttyACM1
UPLOAD_PORT        ?= $(wildcard /dev/ttyUSB*)

#Directory used to build files in, contains all the build files, from object files to the final hex file
#on linux it is best to put an absolute path like /home/username/tmp .
BUILD_DIR          ?= applet

############################################################################

# Be sure to regenerate speed_lookuptable.h with create_speed_lookuptable.py
# if you are setting this to something other than 16MHz
# Set to 16Mhz if not yet set.
F_CPU ?= 16000000

TOOL_PREFIX = avr
MCU_FLAGS   = -mmcu=$(MCU)
SIZE_FLAGS  = --mcu=$(MCU) -C

# Arduino contained the main source code for the Arduino
# Libraries, the "hardware variant" are for boards
# that derives from that, and their source are present in
# the main Marlin source directory

TARGET = $(notdir $(CURDIR))

# VPATH tells make to look into these directory for source files,
# there is no need to specify explicit pathnames as long as the
# directory is added here

# The Makefile for previous versions of Marlin used VPATH for all
# source files, but for Marlin 2.0, we use VPATH only for arduino
# library files.

VPATH = .
VPATH += $(BUILD_DIR)
VPATH += $(HARDWARE_SRC)
VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/cores/arduino
VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SPI
VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SPI/src
VPATH += $(ARDUINO_INSTALL_DIR)/hardware/arduino/avr/libraries/SoftwareSerial/src
VPATH += $(ARDUINO_INSTALL_DIR)/libraries/LiquidCrystal/src

LIB_SRC = wiring.c \
  wiring_analog.c wiring_digital.c \
  wiring_shift.c WInterrupts.c hooks.c wiring_pulse.c

LIB_CXXSRC = WMath.cpp WString.cpp Print.cpp SPI.cpp

LIB_CXXSRC += LiquidCrystal.cpp

#Check for Arduino 1.0.0 or higher and use the correct source files for that version
ifeq ($(shell [ $(ARDUINO_VERSION) -ge 100 ] && echo true), true)
  LIB_CXXSRC += main.cpp
else
  LIB_SRC += pins_arduino.c main.c
endif

FORMAT = ihex

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

OPT = s

DEFINES ?=

# Program settings
CC = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-gcc
CXX = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-g++
OBJCOPY = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-objdump
AR  = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-ar
SIZE = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-size
NM = $(AVR_TOOLS_PATH)$(TOOL_PREFIX)-nm
AVRDUDE = $(ARDUINO_INSTALL_DIR)/hardware/tools/avr/bin/avrdude
REMOVE = rm -f
MV = mv -f

# Place -D or -U options here
CDEFS    = -DF_CPU=$(F_CPU) ${addprefix -D , $(DEFINES)} -DARDUINO=$(ARDUINO_VERSION)
CXXDEFS  = $(CDEFS)

# Add all the source directories as include directories too
CINCS = ${addprefix -I ,${VPATH}}
CXXINCS = ${addprefix -I ,${VPATH}}

# Silence warnings for library code (won't work for .h files, unfortunately)
LIBWARN = -w -Wno-packed-bitfield-compat

# Compiler flag to set the C/CPP Standard level.
CSTANDARD = -std=gnu99
CXXSTANDARD = -std=gnu++11
CDEBUG = -g$(DEBUG)
CWARN   = -Wall -Wstrict-prototypes -Wno-packed-bitfield-compat -Wno-pragmas -Wunused-parameter
CXXWARN = -Wall                     -Wno-packed-bitfield-compat -Wno-pragmas -Wunused-parameter
CTUNING = -fsigned-char -funsigned-bitfields -fno-exceptions \
          -fshort-enums -ffunction-sections -fdata-sections

CTUNING += -DMOTHERBOARD=${HARDWARE_MOTHERBOARD}

CXXEXTRA = -fno-use-cxa-atexit -fno-threadsafe-statics -fno-rtti
CFLAGS := $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CEXTRA)   $(CTUNING) $(CSTANDARD)
CXXFLAGS :=         $(CDEFS) $(CINCS) -O$(OPT) $(CXXEXTRA) $(CTUNING) $(CXXSTANDARD)
ASFLAGS :=          $(CDEFS)

LD_PREFIX = -Wl,--gc-sections,--relax
LDFLAGS   = -lm
CTUNING   += -flto

# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(UPLOAD_PORT)
# -V means do not verify... just for protos
AVRDUDE_WRITE_FLASH =  -V -Uflash:w:$(BUILD_DIR)/$(TARGET).hex:i
AVRDUDE_CONF = $(ARDUINO_INSTALL_DIR)/hardware/tools/avr/etc/avrdude.conf
AVRDUDE_FLAGS = -D -C$(AVRDUDE_CONF) \
  -p$(MCU) -P$(AVRDUDE_PORT) -c$(AVRDUDE_PROGRAMMER) \
  -b$(UPLOAD_RATE)

# Since Marlin 2.0, the source files may be distributed into several
# different directories, so it is necessary to find them recursively

SRC    = $(shell find src -name '*.c'   -type f)
CXXSRC = $(shell find src -name '*.cpp' -type f)

# Define all object files.
OBJ  = ${patsubst %.c,   $(BUILD_DIR)/arduino/%.o, ${LIB_SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/arduino/%.o, ${LIB_CXXSRC}}
OBJ += ${patsubst %.S,   $(BUILD_DIR)/arduino/%.o, ${LIB_ASRC}}
OBJ += ${patsubst %.c,   $(BUILD_DIR)/%.o, ${SRC}}
OBJ += ${patsubst %.cpp, $(BUILD_DIR)/%.o, ${CXXSRC}}

# Define all listing files.
LST = $(LIB_ASRC:.S=.lst) $(LIB_CXXSRC:.cpp=.lst) $(LIB_SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS   = $(MCU_FLAGS) $(CPU_FLAGS) $(CFLAGS) -I.
ALL_CXXFLAGS = $(MCU_FLAGS) $(CPU_FLAGS) $(CXXFLAGS)
ALL_ASFLAGS  = $(MCU_FLAGS) $(CPU_FLAGS) $(ASFLAGS) -x assembler-with-cpp

# set V=1 (eg, "make V=1") to print the full commands etc.
ifneq ($V,1)
  Pecho=@echo
  P=@
else
  Pecho=@:
  P=
endif

# Create required build hierarchy if it does not exist

$(shell mkdir -p $(dir $(OBJ)))

# Default target.
all: sizeafter

build: elf hex bin

elf: $(BUILD_DIR)/$(TARGET).elf
bin: $(BUILD_DIR)/$(TARGET).bin
hex: $(BUILD_DIR)/$(TARGET).hex
eep: $(BUILD_DIR)/$(TARGET).eep
lss: $(BUILD_DIR)/$(TARGET).lss
sym: $(BUILD_DIR)/$(TARGET).sym

# Program the device.
# Do not try to reset an Arduino if it's not one
upload:
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty hup < $(UPLOAD_PORT); true
endif
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
ifeq (${AVRDUDE_PROGRAMMER}, arduino)
	stty -hup < $(UPLOAD_PORT); true
endif
	sudo minicom -D $(UPLOAD_PORT)

upload_stock:
	stty hup < $(UPLOAD_PORT); true
	$(AVRDUDE) $(AVRDUDE_FLAGS) -Uflash:w:A8-S_firmwareUpdate.hex:i
	stty -hup < $(UPLOAD_PORT); true

avrdude:
	stty hup < $(UPLOAD_PORT); true
	$(AVRDUDE) $(AVRDUDE_FLAGS) -t
	stty -hup < $(UPLOAD_PORT); true

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILD_DIR)/$(TARGET).hex
ELFSIZE = $(SIZE)  $(SIZE_FLAGS) $(BUILD_DIR)/$(TARGET).elf; \
          $(SIZE)  $(BUILD_DIR)/$(TARGET).elf
sizebefore:
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter: build
	$P if [ -f $(BUILD_DIR)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi

.SUFFIXES: .elf .hex .eep .lss .sym .bin
.PRECIOUS: .o

.elf.hex:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.bin:
	$(Pecho) "  COPY  $@"
	$P $(OBJCOPY) -O binary -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
    --change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

# Link: create ELF output file from library.

$(BUILD_DIR)/$(TARGET).elf: $(OBJ) Configuration.h
	$(Pecho) "  CXX   $@"
	$P $(CC) $(LD_PREFIX) $(ALL_CXXFLAGS) -o $@ -L. $(OBJ) $(LDFLAGS) $(LD_SUFFIX)

# Object files that were found in "src" will be stored in $(BUILD_DIR)
# in directories that mirror the structure of "src"

$(BUILD_DIR)/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $(CWARN) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS) $(CXXWARN) $< -o $@

# Object files for Arduino libs will be created in $(BUILD_DIR)/arduino

$(BUILD_DIR)/arduino/%.o: %.c Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CC    $<"
	$P $(CC) -MMD -c $(ALL_CFLAGS) $(LIBWARN) $< -o $@

$(BUILD_DIR)/arduino/%.o: %.cpp Configuration.h Configuration_adv.h $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_CXXFLAGS)  $(LIBWARN) $< -o $@

$(BUILD_DIR)/arduino/%.o: %.S $(MAKEFILE)
	$(Pecho) "  CXX   $<"
	$P $(CXX) -MMD -c $(ALL_ASFLAGS) $< -o $@

# Target: clean project.
clean:
	$(Pecho) "  RMDIR $(BUILD_DIR)/"
	$P rm -rf $(BUILD_DIR)


.PHONY: all build elf hex eep lss sym program clean depend sizebefore sizeafter

# Automaticaly include the dependency files created by gcc
-include ${patsubst %.o, %.d, ${OBJ}}

test:
	$(foreach var,$(.VARIABLES),$(info $(var) = $($(var))))
