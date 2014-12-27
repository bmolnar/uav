#AUTOMAKE_OPTIONS = foreign subdir-objects
AUTOMAKE_OPTIONS = foreign

lib_LIBRARIES = libcore.a libWire.a libSPI.a libTinyGPS.a libLiquidCrystal.a libxstream.a
bin_PROGRAMS = test.hex
#bin_PROGRAMS = main.elf main.eep main.hex
#bin_PROGRAMS += test.hex testMPU6050.hex receiver.hex transmitter.hex

CFLAGS = -g -Os -Wall -ffunction-sections -fdata-sections -mmcu=atmega328p
CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p
CPPFLAGS = -MMD -DF_CPU=16000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=101 -I/usr/share/arduino/hardware/arduino/variants/standard
LDFLAGS = -Os -Wl,--gc-sections -mmcu=atmega328p

# libcore
libcore_adir = $(includedir)
libcore_a_SRCDIR = /usr/share/arduino/hardware/arduino/cores/arduino
libcore_a_CPPFLAGS = -I$(libcore_a_SRCDIR)
libcore_a_SOURCES = \
	$(libcore_a_SRCDIR)/wiring_digital.c \
	$(libcore_a_SRCDIR)/wiring_pulse.c \
	$(libcore_a_SRCDIR)/wiring.c \
	$(libcore_a_SRCDIR)/WInterrupts.c \
	$(libcore_a_SRCDIR)/wiring_analog.c \
	$(libcore_a_SRCDIR)/wiring_shift.c \
	$(libcore_a_SRCDIR)/HID.cpp \
	$(libcore_a_SRCDIR)/Stream.cpp \
	$(libcore_a_SRCDIR)/Tone.cpp \
	$(libcore_a_SRCDIR)/WString.cpp \
	$(libcore_a_SRCDIR)/USBCore.cpp \
	$(libcore_a_SRCDIR)/WMath.cpp \
	$(libcore_a_SRCDIR)/new.cpp \
	$(libcore_a_SRCDIR)/main.cpp \
	$(libcore_a_SRCDIR)/IPAddress.cpp \
	$(libcore_a_SRCDIR)/CDC.cpp \
	$(libcore_a_SRCDIR)/HardwareSerial.cpp \
	$(libcore_a_SRCDIR)/Print.cpp

# libWire
libWire_adir = $(includedir)
libWire_a_SRCDIR = /usr/share/arduino/libraries/Wire
libWire_a_CPPFLAGS = -I$(libWire_a_SRCDIR) -I$(libWire_a_SRCDIR)/utility -I$(libcore_a_SRCDIR)
libWire_a_SOURCES = $(libWire_a_SRCDIR)/Wire.cpp $(libWire_a_SRCDIR)/utility/twi.c

# libSPI
libSPI_adir = $(includedir)
libSPI_a_SRCDIR = /usr/share/arduino/libraries/SPI
libSPI_a_CPPFLAGS = -I$(libSPI_a_SRCDIR) -I$(libcore_a_SRCDIR)
libSPI_a_SOURCES = $(libSPI_a_SRCDIR)/SPI.cpp

# libTinyGPS
libTinyGPS_adir = $(includedir)
libTinyGPS_a_SRCDIR = $(srcdir)/lib/TinyGPS
libTinyGPS_a_CPPFLAGS = -I$(libTinyGPS_a_SRCDIR) -I$(libTinyGPS_a_SRCDIR)/utility -I$(libcore_a_SRCDIR)
libTinyGPS_a_SOURCES = $(libTinyGPS_a_SRCDIR)/TinyGPS.cpp

# libLiquidCrystal
libLiquidCrystal_adir = $(includedir)
libLiquidCrystal_a_SRCDIR = /usr/share/arduino/libraries/LiquidCrystal
libLiquidCrystal_a_CPPFLAGS = -I$(libLiquidCrystal_a_SRCDIR) -I$(libcore_a_SRCDIR)
libLiquidCrystal_a_SOURCES = $(libLiquidCrystal_a_SRCDIR)/LiquidCrystal.cpp

# libxstream
libxstream_adir = $(includedir)
libxstream_a_SRCDIR = $(srcdir)/lib/xstream
libxstream_a_CPPFLAGS = -I$(libxstream_a_SRCDIR)
libxstream_a_SOURCES = \
	$(libxstream_a_SRCDIR)/xprintx.c \
	$(libxstream_a_SRCDIR)/xstream.c

# test.hex
test_hex_SOURCES = src/test.cpp src/HMC5883L.cpp src/MPU9150.cpp src/MPU6050.cpp src/I2C.cpp src/Logger.cpp
test_hex_CPPFLAGS = -I$(srcdir)/src -I$(libxstream_a_SRCDIR) -I$(libLiquidCrystal_a_SRCDIR) -I$(libTinyGPS_a_SRCDIR) -I$(libSPI_a_SRCDIR) -I$(libWire_a_SRCDIR) -I$(libcore_a_SRCDIR)
test_hex_LDADD = libxstream.a libLiquidCrystal.a libTinyGPS.a libSPI.a libWire.a libcore.a
test.hex$(EXEEXT): $(test_hex_OBJECTS) $(test_hex_DEPENDENCIES) $(EXTRA_test_hex_DEPENDENCIES)
	@rm -f $@; rm -f $(@:.hex=.elf); rm -f $(@:.hex=.eep)
	$(CXXLINK) -o $(@:.hex=.elf) $(test_hex_OBJECTS) $(test_hex_LDADD) $(LIBS)
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(@:.hex=.elf) $(@:.hex=.eep)
	avr-objcopy -O ihex -R .eeprom $(@:.hex=.elf) $@



build: all-am
.PHONY: build

UPLOAD_DEVICE = /dev/ttyACM0
upload:
	avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -carduino -P$(UPLOAD_DEVICE) -b115200 -D -Uflash:w:test.hex:i
.PHONY: upload

console:
	minicom -b 38400 -D $(UPLOAD_DEVICE)
.PHONY: console

deploy: build upload console
.PHONY: deploy

