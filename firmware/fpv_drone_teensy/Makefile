# Teensyduino Makefile - minimal

# Modify ARDUINO_PATH to your local Teensyduino install
ARDUINO_PATH ?= /opt/arduino-1.8.19
TEENSY_LOADER_CLI ?= teensy_loader_cli
BOARD ?= teensy41

SRC := $(wildcard *.cpp)
OBJS := $(SRC:.cpp=.o)
INCLUDES := -I. -I$(ARDUINO_PATH)/hardware/teensy/avr/cores/teensy4

# Add CMSIS-DSP include path (installed by Teensy core)
INCLUDES += -I$(ARDUINO_PATH)/hardware/teensy/avr/cores/teensy4/CMSIS/DSP/Include

CXXFLAGS := -std=gnu++17 -O2 -ffunction-sections -fdata-sections -DUSB_SERIAL
LDFLAGS := -Wl,--gc-sections

TARGET = fpv_drone_teensy.hex

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo Linking $@
	@$(ARDUINO_PATH)/hardware/tools/arm/bin/arm-none-eabi-g++ $(OBJS) $(LDFLAGS) -o fpv_drone_teensy.elf
	@$(ARDUINO_PATH)/hardware/tools/arm/bin/arm-none-eabi-objcopy -O ihex -R .eeprom fpv_drone_teensy.elf $@

%.o: %.cpp
	@echo Compiling $<
	@$(ARDUINO_PATH)/hardware/tools/arm/bin/arm-none-eabi-g++ $(CXXFLAGS) $(INCLUDES) -c $< -o $@

upload: $(TARGET)
	$(TEENSY_LOADER_CLI) -w -v -s $(TARGET)

clean:
	rm -f $(OBJS) fpv_drone_teensy.elf $(TARGET) 