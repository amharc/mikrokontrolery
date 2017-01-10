#include "leds.h"
#include "usart.h"
#include "button.h"
#include "i2c.h"
#include "timer.h"
#include "accel.h"

#define CMD_LENGTH 7

int main() {
    init_leds();
    init_usart();
    //init_buttons();
    init_timer();
    init_i2c();
    init_accel();

    set_led(LED_GREEN2, 1);

    for(;;) {
        uint8_t x = 0, y = 0, z = 0;

        output("Read: ");
        output_int(x);
        output(", ");
        output_int(y);
        output(", ");
        output_int(z);
        output("\r\n");

        Delay(100000000);
    }
}
