#include <avr/io.h>
#include <util/delay.h>

#include "i2c.h"

#define PCA9685_ADDRESS 0b1000000

#define OSC_CLOCK 25000000
#define PRE_SCALE(x) OSC_CLOCK / 4096 / (x) - 1

#define PCA9685_REG_MODE1 0x00
#define REG_MODE1_ALLCALL 0
#define REG_MODE1_SUB3 1
#define REG_MODE1_SUB2 2
#define REG_MODE1_SUB1 3
#define REG_MODE1_SLEEP 4
#define REG_MODE1_AI 5
#define REG_MODE1_EXTCLK 6
#define REG_MODE1_RESTART 7

#define PCA9685_REG_MODE2 0x01
#define REG_MODE2_OUTNE 0
#define REG_MODE2_OUTDRV 2
#define REG_MODE2_OCH 3
#define REG_MODE2_INVRT 4

#define PCA9685_REG_PRE_SCALE 0xFE

#define PCA9685_REG_LED0_ON_L 0x06
#define PCA9685_REG_LED0_ON_H 0x07
#define PCA9685_REG_LED0_OFF_L 0x08
#define PCA9685_REG_LED0_OFF_H 0x09

void PCA9685_write_reg(uint8_t reg, uint8_t value)
{
    i2c_start();
    i2c_write((PCA9685_ADDRESS << 1) | I2C_WRITE);
    i2c_write(reg);
    i2c_write(value);
    i2c_stop();
}

void PCA9685_write_buf(uint8_t addr, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write((PCA9685_ADDRESS << 1) | I2C_WRITE);
    i2c_write(addr);
    while (len--)
        i2c_write(*buf++);
    i2c_stop();
}

uint8_t PCA9685_read_reg(uint8_t reg)
{
    uint8_t result;
    i2c_start();
    i2c_write((PCA9685_ADDRESS << 1) | I2C_WRITE);
    i2c_write(reg);
    i2c_start();
    i2c_write((PCA9685_ADDRESS << 1) | I2C_READ);
    result = i2c_read(I2C_NACK);
    i2c_stop();

    return result;
}

void PCA9685_oscillator_on()
{
    uint8_t mode1 = PCA9685_read_reg(PCA9685_REG_MODE1);
    mode1 &= ~(1 << REG_MODE1_SLEEP);
    mode1 |= (1 << REG_MODE1_RESTART);
    PCA9685_write_reg(PCA9685_REG_MODE1, mode1);
    _delay_us(500); // Maximum time for internal oscillator to turn on
}

void PCA9685_oscillator_off()
{
    PCA9685_write_reg(PCA9685_REG_MODE1, (1 << REG_MODE1_SLEEP));
}

void PCA9685_set_frequency(uint16_t freq)
{
    uint8_t presc = PRE_SCALE(freq);

    PCA9685_oscillator_off();
    PCA9685_write_reg(PCA9685_REG_PRE_SCALE, presc);
    PCA9685_oscillator_on();
}

void PCA9685_init()
{
    PCA9685_set_frequency(50);

    // Set AutoIncrement for future width commands
    // Using AI, PWM for whole channel can be done with one transaction
    uint8_t mode1 = PCA9685_read_reg(PCA9685_REG_MODE1);
    PCA9685_write_reg(PCA9685_REG_MODE1, mode1 | (1 << REG_MODE1_AI));
}

void PCA9685_set_width(uint8_t channel, uint16_t on, uint16_t off)
{
#define OFFSET 4

    uint8_t address = PCA9685_REG_LED0_ON_L + OFFSET * channel;
    uint8_t buf[4] = {on & 0xFF, (on >> 8) & 0b00001111, off & 0xFF, (off >> 8) & 0b00001111};
    PCA9685_write_buf(address, buf, 4);
}

int main()
{
    _delay_ms(2000);

    i2c_init();
    PCA9685_init();

    while (1)
    {
        PCA9685_set_width(0, 4095, 4095 / 10);
        PCA9685_set_width(7, 4095, 4095 / 10);
        _delay_ms(2500);
        PCA9685_set_width(0, 4095, 4095 / 20);
        PCA9685_set_width(7, 4095, 4095 / 20);
        _delay_ms(2500);
    }

    return -1;
}