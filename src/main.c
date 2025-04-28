/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define POWER_STATUS_LED 10
#define CLEAN_TANK_LED 11
#define DIRTY_TANK_LED 12
#define ERROR_LED 13
#define SUPER_SONIC_TRIG 28

#define SUPER_SONIC_ECHO 27
#define POWER_STATUS_INPUT 15
#define CLEAN_TANK_INPUT 1
#define ERROR_INPUT 0

#define BIT(n) (1u<<(n))
#define LED_MASK (BIT(POWER_STATUS_LED) | BIT(CLEAN_TANK_LED) | BIT(DIRTY_TANK_LED) | BIT(ERROR_LED)) 
#define INPUT_MASK (BIT(POWER_STATUS_INPUT) | BIT(CLEAN_TANK_INPUT) | BIT(ERROR_INPUT))

#define ON 1
#define OFF 0

void send_trigger_pulse() {
    gpio_put(SUPER_SONIC_TRIG, ON);
    sleep_us(10);
    gpio_put(SUPER_SONIC_TRIG, OFF);
}

double get_distance_cm() {
    // Send trigger
    send_trigger_pulse();

    // echo start
    while (gpio_get(SUPER_SONIC_ECHO) == 0);

    absolute_time_t start_time = get_absolute_time(); // pico sdk function

    // Wait for echo end
    while (gpio_get(SUPER_SONIC_ECHO) == 1);

    absolute_time_t end_time = get_absolute_time();

    // Calculate time difference in microseconds
    int64_t time_diff_us = absolute_time_diff_us(start_time, end_time);

    // Speed of sound is ~343 m/s, or 0.0343 cm/us
    // Distance = (time * speed of sound) / 2 (round trip)
    float distance_cm = (time_diff_us * 0.0343f) / 2.0f;

    return distance_cm;
}

int main() {
    gpio_init_mask(LED_MASK);
    gpio_set_dir_out_masked(LED_MASK); // Initialize output signals (LEDs)
    gpio_init_mask(INPUT_MASK); // Initialize input signals
    gpio_set_dir_in_masked(INPUT_MASK); 

    // Initialize trigger pin
    gpio_init(SUPER_SONIC_TRIG);
    gpio_set_dir(SUPER_SONIC_TRIG, GPIO_OUT);

    // Initialize echo pin
    gpio_init(SUPER_SONIC_ECHO);
    gpio_set_dir(SUPER_SONIC_ECHO, GPIO_IN);

    int PROGRAM_STATE = OFF;

    while (true) {
        if (gpio_get(POWER_STATUS_INPUT) == ON) {
          PROGRAM_STATE = ON;
          gpio_put(POWER_STATUS_LED, ON);
        }
        while (PROGRAM_STATE == ON) {
          int DIRTY_TANK_STATUS = OFF;
          if (get_distance_cm() < 50) DIRTY_TANK_STATUS = ON;
          gpio_put(CLEAN_TANK_LED, gpio_get(CLEAN_TANK_INPUT));
          gpio_put(DIRTY_TANK_LED, DIRTY_TANK_STATUS);
          gpio_put(ERROR_LED, gpio_get(ERROR_INPUT));
        }
    }
}
