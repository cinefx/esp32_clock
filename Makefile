###############################################################################
# Makefile for the project ScopeClock
###############################################################################

## General Flags
PROJECT = ScopeClock
MCU = atmega328p
TARGET = ScopeClock.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                            -fno-inline-small-functions                 -DF_CPU=20000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-relax -Wl,-Map=ScopeClock.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Electronics\DutchtronixClock\Version 4.0 Frozen 10-04-10\SourceCodeRelease\i2cmaster" 

## Objects that must be built in order to link
OBJECTS = twimaster.o ClkRender.o ClkISR.o ClkSupport.o ClkDSTime.o ClkTerm.o ClkGen.o ClkDebug.o ClkApp.o ClkCmd.o ClkData.o ClkDemo.o ClkMenu.o ClkGPS.o ClkFlash.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) ScopeClock.hex ScopeClock.eep ScopeClock.lss size

## Compile
twimaster.o: ../i2cmaster/twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkRender.o: ../ClkRender.s
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

ClkISR.o: ../ClkISR.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkSupport.o: ../ClkSupport.c
	$(CC) $(INCLUDES) $(CFLAGS) -c -O1 $<

ClkDSTime.o: ../ClkDSTime.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkTerm.o: ../ClkTerm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkGen.o: ../ClkGen.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkDebug.o: ../ClkDebug.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkApp.o: ../ClkApp.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkCmd.o: ../ClkCmd.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkData.o: ../ClkData.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkDemo.o: ../ClkDemo.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkMenu.o: ../ClkMenu.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkGPS.o: ../ClkGPS.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ClkFlash.o: ../ClkFlash.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) ScopeClock.elf dep/* ScopeClock.hex ScopeClock.eep ScopeClock.lss ScopeClock.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

