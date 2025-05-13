/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#define POWER_STATUS_LED 18
#define CLEAN_TANK_LED 17
#define DIRTY_TANK_LED 20
#define ERROR_LED 19
#define ULTRASONIC_TRIG 6

#define ULTRASONIC_ECHO 1
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

int PROGRAM_STATE = OFF;
int ERROR_FLAG = OFF;
int CLEANING_SETTING = 1; // Low = 0, Med = 1, High = 2

void send_trigger_pulse() {
    gpio_put(ULTRASONIC_TRIG, ON);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    gpio_put(ULTRASONIC_TRIG, OFF);
}

double get_distance_cm() {
    send_trigger_pulse();

    absolute_time_t timeout = make_timeout_time_ms(200);

    // Wait for echo to go HIGH
    while (gpio_get(ULTRASONIC_ECHO) == 0) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            printf("Echo start timeout\n");
            return -1.0;
        }
    }
    absolute_time_t start_time = get_absolute_time();

    timeout = make_timeout_time_ms(200);
    // Wait for echo to go LOW
    while (gpio_get(ULTRASONIC_ECHO) == 1) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            printf("Echo end timeout\n");
            return -1.0;
        }
    }
    absolute_time_t end_time = get_absolute_time();

    int64_t time_diff_us = absolute_time_diff_us(start_time, end_time);
    float distance_cm = (time_diff_us * 0.0343f) / 2.0f;

    printf("distance: %.2f cm\n", distance_cm);
    return distance_cm;
}

/* FreeRTOS Tasks*/
void PowerTask(void* param) {
    while (1) {
        gpio_put(POWER_STATUS_LED, PROGRAM_STATE);
        if (gpio_get(POWER_STATUS_INPUT) == 1) PROGRAM_STATE = !PROGRAM_STATE;
        vTaskDelay(5);
    }
}

void DirtyTankTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON) {
            int DIRTY_TANK_STATUS = OFF;
            if (get_distance_cm >= 0 && get_distance_cm() < 10) DIRTY_TANK_STATUS = ON;
            gpio_put(DIRTY_TANK_LED, DIRTY_TANK_STATUS);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void CleanTankTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON) {
            gpio_put(CLEAN_TANK_LED, gpio_get(CLEAN_TANK_INPUT));
        }
        vTaskDelay(10);
    }
}

void ErrorTask(void* param) {
    while (1) {
        int ERROR_STATUS = gpio_get(ERROR_INPUT);
        gpio_put(ERROR_LED, gpio_get(ERROR_INPUT));
        if (ERROR_STATUS == ON) {
            ERROR_FLAG = ON;
        }
        vTaskDelay(5);
    }
}

void CleaningSettingTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON && gpio_get(CLEANING_SETTING_INPUT) == 1) {
            CLEANING_SETTING++;
            if (CLEANING_SETTING > 3) CLEANING_SETTING = 1;
            printf("Cleaning Setting: %d\n", CLEANING_SETTING);
            vTaskDelay(100 / portTICK_PERIOD_MS); // to deal with debounce
        }
        vTaskDelay(5);
    }
}

void CleanTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON && gpio_get(START_INPUT) == 1) {
            printf("Starting cleaning...\n");

            gpio_put(RELAY_1_INPUT, ON);
            vTaskDelay(5000 * CLEANING_SETTING);
            gpio_put(RELAY_1_INPUT, OFF);

            gpio_put(RELAY_2_INPUT, ON);
            vTaskDelay(5000 * CLEANING_SETTING);
            gpio_put(RELAY_2_INPUT, OFF);

            gpio_put(RELAY_3_INPUT, ON);
            vTaskDelay(5000 * CLEANING_SETTING);
            gpio_put(RELAY_3_INPUT, OFF);

            gpio_put(RELAY_4_INPUT, ON);
            vTaskDelay(5000 * CLEANING_SETTING);
            gpio_put(RELAY_4_INPUT, OFF);

            printf("Done cleaning!\n");
        }
        vTaskDelay(10); 
    }
}

int main() {
    stdio_init_all();
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

    TaskHandle_t powTask = NULL;
    TaskHandle_t dTankTask = NULL;
    TaskHandle_t cTankTask = NULL;
    TaskHandle_t errTask = NULL;
    TaskHandle_t settingTask = NULL;
    TaskHandle_t cleanTask = NULL;

    uint32_t status = xTaskCreate(
        PowerTask,
        "Manage program state",
        1024,
        NULL,
        4,
        &powTask);

    status = xTaskCreate(
        DirtyTankTask,
        "Manage dirty tank sensor and indicator",
        1024,
        NULL,
        1,
        &dTankTask        
    );

    status = xTaskCreate(
        CleanTankTask,
        "Manage clean tank sensor and indicator",
        1024,
        NULL,
        1,
        &cTankTask        
    );

    status = xTaskCreate(
        ErrorTask,
        "Manage erroring",
        1024,
        NULL,
        3,
        &errTask        
    );

    status = xTaskCreate(
        CleaningSettingTask,
        "Manage user input for cleaning setting",
        1024,
        NULL,
        2,
        &settingTask
    );

    status = xTaskCreate(
        CleanTask,
        "Manage the cleaning operation",
        1024,
        NULL,
        4,
        &cleanTask
    );

    vTaskStartScheduler();

    while (1) { // Should never reach this 
    }
}
