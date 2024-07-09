#include <avr/io.h>
#include <util/delay.h>

#include "pca9685.h"

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