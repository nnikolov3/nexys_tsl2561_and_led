# Makefile for ECE 544 Project #2 - TSL2561 and LED PID Control
# Targets MicroBlaze with FreeRTOS using clang

# Compiler settings
CC = clang
CFLAGS = -g -Wall -Wextra -std=c11 -I.  # Debug, warnings, C11, include current dir
LDFLAGS = -lm  # Link math library (if needed)

# FreeRTOS and MicroBlaze-specific flags (adjust as needed)
# Note: For MicroBlaze, replace clang with mb-gcc or specify -target microblaze-xilinx-elf
# CFLAGS += -target microblaze-xilinx-elf -mxl-barrel-shift -mxl-pattern-compare
# LDFLAGS += -L/path/to/freertos/lib -lfreertos

# Source files
SOURCES = FreeRtosDemo.c main.c platform.c tsl2561.c
HEADERS = main.h platform_config.h platform.h tsl2561.h

# Object files (generated from .c files)
OBJECTS = $(SOURCES:.c=.o)

# Target executable
TARGET = nexys_tsl2561_and_led

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compile .c files to .o files
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJECTS) $(TARGET)

# Phony targets
.PHONY: all clean