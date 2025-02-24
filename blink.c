/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#define LED_DELAY_MS 250
#define LED_PIN 25
int led_value = 0;

void toggle_LED() {
    led_value = 1 - led_value;
    gpio_put(LED_PIN, led_value);
    printf("LED TOGGLED \n");
}

// Executes each time core 0 sends information to core 1 through the FIFO buffer
void core1_interrupt_handler() {
    while(multicore_fifo_rvalid()) {
        int data = multicore_fifo_pop_blocking();
        printf("Data Received from core 0: %d\n", data);
    }
    multicore_fifo_clear_irq();
}

// Core 1 Main Code
void core1_entry() {
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    // infinite loop which waits for interrupt
    while(1) {
        tight_loop_contents();
    }
}


// Core 0 Main Code
int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    multicore_launch_core1(core1_entry);

    // loop to send info to core 1
    int x = 0;
    while(1) {
        multicore_fifo_push_blocking(x);
        toggle_LED();
        sleep_ms(3000);
    }

}
