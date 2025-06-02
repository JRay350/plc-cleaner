/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define HX711_NO_MUTEX
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/binary_info.h"
#include "ow_rom.h"
#include "ds18b20.h"

#include "extern/onewire_library/onewire_library.h"
#include "extern/hx711-pico-c/include/common.h"

#define PRESSURE_SCL 17
#define PRESSURE_SDA 16

#define POWER_STATUS_LED 18
#define CLEAN_TANK_LED 22
#define DIRTY_TANK_LED 20
#define ERROR_LED 19
#define CLEANING_SETTING_LED 0

#define ULTRASONIC_TRIG 6
#define ULTRASONIC_ECHO 1

#define POWER_STATUS_INPUT 14
#define CLEAN_TANK_INPUT 27
#define ERROR_INPUT 10
#define DIRTY_TANK_INPUT 13

#define CLEANING_SETTING_INPUT 11
#define START_INPUT 9

#define RELAY_1_INPUT 2
#define RELAY_2_INPUT 3
#define RELAY_3_INPUT 4
#define RELAY_4_INPUT 5

#define PRESSURE_OUT 8
#define PRESSURE_CLK 7
#define TEMP_DATA 21


#define BIT(n) (1u<<(n))
#define LED_MASK (BIT(POWER_STATUS_LED) | BIT(CLEAN_TANK_LED) | BIT(DIRTY_TANK_LED) | BIT(ERROR_LED) | BIT(CLEANING_SETTING_LED)) 
#define INPUT_MASK (BIT(POWER_STATUS_INPUT) | BIT(CLEAN_TANK_INPUT) | BIT(ERROR_INPUT) | BIT(DIRTY_TANK_INPUT) | BIT(CLEANING_SETTING_INPUT))
#define RELAY_MASK (BIT(RELAY_1_INPUT) | BIT(RELAY_2_INPUT) | BIT(RELAY_3_INPUT) | BIT(RELAY_4_INPUT))

#define ON 1
#define OFF 0

int PROGRAM_STATE = OFF;
int DIRTY_TANK_ERROR = OFF;
int CLEAN_TANK_ERROR = OFF;
int PRESSURE_ERROR = OFF;
int CLEANING_SETTING = 1; // Low = 1, Med = 2, High = 3

TaskHandle_t powTask = NULL;
TaskHandle_t dTankTask = NULL;
TaskHandle_t cTankTask = NULL;
TaskHandle_t errTask = NULL;
TaskHandle_t settingTask = NULL;
TaskHandle_t cleanSettingLEDTask = NULL;
TaskHandle_t cleanTask = NULL;
TaskHandle_t pressureTask = NULL;

void SendTriggerPulse() {
    gpio_put(ULTRASONIC_TRIG, ON);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    gpio_put(ULTRASONIC_TRIG, OFF);
}

double GetDistance() {
    SendTriggerPulse();

    absolute_time_t timeout = make_timeout_time_ms(200);

    // Wait for echo to go HIGH
    while (gpio_get(ULTRASONIC_ECHO) == 0) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            //////printf("Echo start timeout\n");
            return -1.0;
        }
    }
    absolute_time_t start_time = get_absolute_time();

    timeout = make_timeout_time_ms(200);
    // Wait for echo to go LOW
    while (gpio_get(ULTRASONIC_ECHO) == 1) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            //////printf("Echo end timeout\n");
            return -1.0;
        }
    }
    absolute_time_t end_time = get_absolute_time();

    int64_t time_diff_us = absolute_time_diff_us(start_time, end_time);
    float distance_cm = (time_diff_us * 0.0343f) / 2.0f;

    //////printf("distance: %.2f cm\n", distance_cm);
    return distance_cm;
}

/* FreeRTOS Tasks*/
void PowerTask(void* param) {
    while (1) {
        if (gpio_get(POWER_STATUS_INPUT) == ON) {
            if (PROGRAM_STATE == ON) { // If power button is pressed when already on
                PROGRAM_STATE = !PROGRAM_STATE;
                gpio_put_masked(LED_MASK, OFF);
            } else { // If power button is pressed when off
                PROGRAM_STATE = !PROGRAM_STATE;
            }
            gpio_put(POWER_STATUS_LED, PROGRAM_STATE);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(5);
    }
}

void DirtyTankTask(void* param) {
    while (1) {
        if (GetDistance() >= 0 && GetDistance() < 8.09625) DIRTY_TANK_ERROR = ON;
        else {
            DIRTY_TANK_ERROR = OFF;
        }
        gpio_put(DIRTY_TANK_LED, DIRTY_TANK_ERROR);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void CleanTankTask(void* param) {
    while (1) {
        if (gpio_get(CLEAN_TANK_INPUT) == ON) {
            CLEAN_TANK_ERROR = ON;
        }
        else {
            CLEAN_TANK_ERROR = OFF;
        }

        gpio_put(CLEAN_TANK_LED, CLEAN_TANK_ERROR);
        vTaskDelay(10);
    }
}


void ErrorTask(void* param) {
    static int isHalted = 0;
    static int causedByError = 0;
    while (1) {
        int isError = DIRTY_TANK_ERROR || CLEAN_TANK_ERROR || PRESSURE_ERROR;
        gpio_put(ERROR_LED, isError);

        if (isError && !isHalted && PROGRAM_STATE == ON) {
            printf("Error, halting operation...\n");
            isHalted = ON;
            PROGRAM_STATE = OFF;
            causedByError = 1;

            gpio_put(ERROR_LED, ON);

            vTaskSuspend(settingTask);
            vTaskSuspend(cleanSettingLEDTask);
            gpio_put(CLEANING_SETTING_LED, OFF);
            vTaskSuspend(cleanTask);
        }

        if (!isError && isHalted) {
            printf("Errors Cleared...\n");
            isHalted = OFF;
            PROGRAM_STATE = ON;
            gpio_put(POWER_STATUS_LED, ON);

            gpio_put(ERROR_LED, OFF);

            vTaskResume(settingTask);
            vTaskResume(cleanSettingLEDTask);
            vTaskResume(cleanTask);
        }
        vTaskDelay(50);
    }
}

void CleaningSettingTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON && gpio_get(CLEANING_SETTING_INPUT) == 1) {
            CLEANING_SETTING++;
            if (CLEANING_SETTING > 3) CLEANING_SETTING = 1;
            printf("Cleaning Setting: %d\n", CLEANING_SETTING);
            vTaskDelay(300 / portTICK_PERIOD_MS); // to deal with debounce
        }
        vTaskDelay(5);
    }
}

void CleaningSettingLED(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON) {
            // blink depending more rapidly at higher cleaning settings
            gpio_put(CLEANING_SETTING_LED, ON);
            vTaskDelay(1000 / CLEANING_SETTING / portTICK_PERIOD_MS);
            gpio_put(CLEANING_SETTING_LED, OFF);
            vTaskDelay(1000 / CLEANING_SETTING / portTICK_PERIOD_MS);
        }
        vTaskDelay(20);
    }
}

void CleanTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON && gpio_get(START_INPUT) == 1) {
            printf("Starting cleaning...\n");

            gpio_put(RELAY_1_INPUT, ON);
            vTaskDelay(5000 * CLEANING_SETTING); // 5 seconds * either 1, 2, or 3 depending on LOW, MED, HIGH
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

            //////printf("Done cleaning!\n");
        }
        vTaskDelay(20); 
    }
}

void PressureTask(void* param) {
    while (1) {
        if (PROGRAM_STATE == ON) {
            // Start measurement
            uint8_t cmd[] = {0xAA, 0x00, 0x00}; // based on protocol from datasheet
            if (i2c_write_blocking(i2c0, 0x18, cmd, 3, false) != 3) {
                printf("Failed to send measurement command\n");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                continue;
            }

            // Wait for conversion to complete (bit 5 == BUSY)
            absolute_time_t start = get_absolute_time();
            uint8_t status = 0x20;
            while (status & 0x20) {
                i2c_read_blocking(i2c0, 0x18, &status, 1, false);
                if (absolute_time_diff_us(start, get_absolute_time()) > 20000) {
                    printf("Sensor timeout\n");
                    status = 0;  // break out
                    break;
                }
            }

            // Read 4 bytes (status + 3 data bytes)
            uint8_t buffer[4];
            if (i2c_read_blocking(i2c0, 0x18, buffer, 4, false) != 4) {
                printf("Failed to read pressure data\n");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                continue;
            }

            // Check error bits
            if (buffer[0] & 0x04 || buffer[0] & 0x01) {
                printf("Sensor error: status = 0x%02X\n", buffer[0]);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                continue;
            }

            // Extract 24-bit raw value
            int32_t raw = ((int32_t)buffer[1] << 16) | ((int32_t)buffer[2] << 8) | buffer[3];

            // Apply transfer function
            float psi = (raw - 1677722) * 25.0f / (15099494 - 1677722) + 0.0f;
            float hpa = psi * 68.9476f;

            if (psi > 18) {
                gpio_put(ERROR_LED, ON);
                printf("Pressure: %.2f hPa (%.2f PSI), which exceeds the limit of 18 PSI\n", hpa, psi);
                PRESSURE_ERROR = ON;
            } else {
                printf("Pressure: %.2f hPa (%.2f PSI)\n", hpa, psi);
                PRESSURE_ERROR = OFF;
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
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

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PRESSURE_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PRESSURE_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PRESSURE_SDA);
    gpio_pull_up(PRESSURE_SCL);

    uint32_t status = xTaskCreate(
        PowerTask,
        "Manage program state",
        1024,
        NULL,
        5,
        &powTask);

    status = xTaskCreate(
        DirtyTankTask,
        "Manage dirty tank sensor and indicator",
        1024,
        NULL,
        4,
        &dTankTask        
    );

    status = xTaskCreate(
        CleanTankTask,
        "Manage clean tank sensor and indicator",
        1024,
        NULL,
        4,
        &cTankTask        
    );

    status = xTaskCreate(
        ErrorTask,
        "Manage erroring",
        1024,
        NULL,
        4,
        &errTask        
    );

    status = xTaskCreate(
        CleaningSettingTask,
        "Manage user input for cleaning setting",
        1024,
        NULL,
        3,
        &settingTask
    );

    status = xTaskCreate(
        CleaningSettingLED,
        "Manage LED for cleaning setting",
        1024,
        NULL,
        2,
        &cleanSettingLEDTask
    );

    status = xTaskCreate(
        CleanTask,
        "Manage the cleaning operation",
        1024,
        NULL,
        5,
        &cleanTask
    );

    status = xTaskCreate(
        PressureTask,
        "Manage pressure sensor",
        1024,
        NULL,
        4,
        &pressureTask
    );

    vTaskStartScheduler();

    while (1) { // should never reach here
    }
}
