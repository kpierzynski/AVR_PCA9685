#ifndef __PCA9685_H_
#define __PCA9685_H_

#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"

#include "pca9685.mem.h"

// Address format in case of this chip is: 1 A5 A4 A3 A2 A1 A0
// where An is physical pin.
#define PCA9685_ADDRESS 0b1000000

#define OSC_CLOCK 25000000
#define PRE_SCALE(x) OSC_CLOCK / 4096 / (x) - 1

void PCA9685_write_reg(uint8_t reg, uint8_t value);
void PCA9685_write_buf(uint8_t addr, uint8_t *buf, uint8_t len);
uint8_t PCA9685_read_reg(uint8_t reg);

void PCA9685_oscillator_on();
void PCA9685_oscillator_off();

void PCA9685_set_frequency(uint16_t freq);
void PCA9685_set_width(uint8_t channel, uint16_t on, uint16_t off);

void PCA9685_init();

#endif