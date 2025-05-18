#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"


int main()
{

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
