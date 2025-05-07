/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#define POWER_STATUS_LED 18
#define CLEAN_TANK_LED 17
#define DIRTY_TANK_LED 20
#define ERROR_LED 19
#define ULTRASONIC_TRIG 28

#define ULTRASONIC_ECHO 27
#define POWER_STATUS_INPUT 14
#define CLEAN_TANK_INPUT 12
#define ERROR_INPUT 10
#define DIRTY_TANK_INPUT 13

#define CLEANING_SETTING_INPUT 11
#define START_INPUT 9

#define RELAY_1_INPUT 2
#define RELAY_2_INPUT 3
#define RELAY_3_INPUT 4
#define RELAY_4_INPUT 5

#define BIT(n) (1u<<(n))
#define LED_MASK (BIT(POWER_STATUS_LED) | BIT(CLEAN_TANK_LED) | BIT(DIRTY_TANK_LED) | BIT(ERROR_LED)) 
#define INPUT_MASK (BIT(POWER_STATUS_INPUT) | BIT(CLEAN_TANK_INPUT) | BIT(ERROR_INPUT) | BIT(DIRTY_TANK_INPUT))
#define RELAY_MASK (BIT(RELAY_1_INPUT) | BIT(RELAY_2_INPUT) | BIT(RELAY_3_INPUT) | BIT(RELAY_4_INPUT))

#define ON 1
#define OFF 0

void send_trigger_pulse() {
    gpio_put(ULTRASONIC_TRIG, ON);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG, OFF);
}

double get_distance_cm() {
    // Send trigger
    send_trigger_pulse();

    // Echo start
    while (gpio_get(ULTRASONIC_ECHO) == 0);

    absolute_time_t start_time = get_absolute_time(); // Pico sdk function

    // Wait for echo end
    while (gpio_get(ULTRASONIC_ECHO) == 1);

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

    // Initialize mask for relays
    gpio_init_mask(RELAY_MASK);
    gpio_set_dir_out_masked(RELAY_MASK);

    // Initialize trigger pin
    gpio_init(ULTRASONIC_TRIG);
    gpio_set_dir(ULTRASONIC_TRIG, GPIO_OUT);

    // Initialize echo pin
    gpio_init(ULTRASONIC_ECHO);
    gpio_set_dir(ULTRASONIC_ECHO, GPIO_IN);

    int CLEANING_SETTING = 0; // Low = 0, Med = 1, High = 2

    int PROGRAM_STATE = OFF;
    int PREVIOUS_BUTTON_SIGNAL = OFF;

    while (true) {
        int CURRENT_BUTTON_SIGNAL = gpio_get(POWER_STATUS_INPUT);

        // Toggle program's state when power button goes from LOW to HIGH
        if (CURRENT_BUTTON_SIGNAL == ON && PREVIOUS_BUTTON_SIGNAL == OFF) { 
            PROGRAM_STATE = !PROGRAM_STATE;
            gpio_put(POWER_STATUS_LED, PROGRAM_STATE);
        }

        PREVIOUS_BUTTON_SIGNAL = CURRENT_BUTTON_SIGNAL;

        while (PROGRAM_STATE == ON) {
            // Dirty tank I/O using ultrasonic sensor results
            int DIRTY_TANK_STATUS = OFF;
            // if (get_distance_cm() < 50) DIRTY_TANK_STATUS = ON;

            // Other I/O
            gpio_put(CLEAN_TANK_LED, gpio_get(CLEAN_TANK_INPUT));
            gpio_put(DIRTY_TANK_LED, gpio_get(DIRTY_TANK_INPUT));
            gpio_put(ERROR_LED, gpio_get(ERROR_INPUT));
            gpio_set_mask(RELAY_MASK);

            // Check for another LOW to HIGH transition to turn off PROGRAM_STATE
            CURRENT_BUTTON_SIGNAL = gpio_get(POWER_STATUS_INPUT);
            if (CURRENT_BUTTON_SIGNAL == ON && PREVIOUS_BUTTON_SIGNAL == OFF) {
                PROGRAM_STATE = OFF;
                gpio_put_masked(LED_MASK, OFF);
            }

            PREVIOUS_BUTTON_SIGNAL = CURRENT_BUTTON_SIGNAL;
        }
    }
}
