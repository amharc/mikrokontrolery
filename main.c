#include "leds.h"
#include "usart.h"
#include "button.h"
#include "i2c.h"

#define CMD_LENGTH 7

int main() {
    init_leds();
    init_usart();
    //init_buttons();
    init_i2c();

//    set_led(LED_GREEN2, 1);

    for(;;) {
        uint8_t x = 0, y = 0, z = 0;

        if (i2c_accel_read(I2C_ACCEL_REG_X, &x)) {
            output("Failed to read X\r\n");
            continue;
        }

        if (i2c_accel_read(I2C_ACCEL_REG_Y, &y)) {
            output("Failed to read Y\r\n");
            continue;
        }

        if (i2c_accel_read(I2C_ACCEL_REG_Z, &z)) {
            output("Failed to read Z\r\n");
            continue;
        }

        output("Read: ");
        output_int(x);
        output(", ");
        output_int(y);
        output(", ");
        output_int(z);
        output("\r\n");

//        toggle_led(LED_GREEN2);

        Delay(1000000);
    }
}
