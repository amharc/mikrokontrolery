#include "leds.h"
#include "usart.h"
#include "button.h"
#include "i2c.h"
#include "timer.h"
#include "accel.h"

int main() {
    init_leds();
    init_usart();
    init_buttons();
    init_timer();
    init_i2c();
    init_accel();

    set_led(LED_GREEN2, 1);
    return 0;
}
