MCU=atmega328p
F_CPU=8000000

CC=avr-gcc
OBJCOPY=avr-objcopy

CFLAGS=-Wall -g -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.

AVRDUDE_FLAGS=-B 2

PORT=COM13
PROGRAMMER=usbasp
TARGET=main
SRC_DIRS=.
BUILD_DIR=./build

SRCS= $(shell find $(SRC_DIRS) -name '*.c')
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/${TARGET}: $(OBJS)
	${CC} ${CFLAGS} ${OBJS} -o $@.bin
	${OBJCOPY} -j .text -j .data -O ihex $@.bin $@.hex
	avr-size --mcu=${MCU} -C $@.bin

$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	${CC} ${CFLAGS} -c $< -o $@

flash:
	avrdude -p ${MCU} ${AVRDUDE_FLAGS} -c ${PROGRAMMER} -U flash:w:${BUILD_DIR}/${TARGET}.hex:i -P ${PORT}

reset:
	avrdude -p ${MCU} ${AVRDUDE_FLAGS} -c ${PROGRAMMER} -P ${PORT}

.PHONY: clean

clean:
	rm -rf *.bin *.hex *.o ${BUILD_DIR}
