#include "driver/adc_types_legacy.h"
#include "hal/adc_types.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <driver/adc.h>

// WARNING:
// This code is written for ESP32, review this before using the ESP32-C3.
// Especially the pins.

// TODO:
// - Logging the values to persistent storage
//      - We probably need an SD card for that -> r2.
// - Kalman filter
// - I2C display
// - SoftAP/HTTP/(SSE/WS) view values in browser?

#define TAG "fqi"

#define FLOW_SENSOR_PIN GPIO_NUM_4
#define FLOW_CALIBRATION_FACTOR 86400

#define LEVEL_SENSOR_PIN ADC1_CHANNEL_6 // GPIO34
#define LEVEL_CALIBRATION_LOW 0.0
#define LEVEL_CALIBRATION_HIGH 100.0

typedef struct {
    float flow_lph;
    float level;
} sensor_readings_t;
QueueHandle_t readings_queue; // Sensor readings are transferred from the sensor
                              // thread to the display thread by this queue.

// The flow is measured by this counter unit.
// The sensor is of an open drain type, meaning that some kind of pullup is
// neccessary to read it, internal or external.
// One liter flowing through the sensor causes `FLOW_CALIBRATION_FACTOR`
// pulses on the line.
// The value returned by `flow_sensor_read` is in liters per second.
pcnt_unit_handle_t pcnt_unit;
void flow_sensor_init(void)
{
    pcnt_unit_config_t pcnt_unit_cfg = {
        .low_limit = -1,
        .high_limit = 5000,
    };
    pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_cfg, &pcnt_unit));

    pcnt_chan_config_t pcnt_chan_cfg = {
        .edge_gpio_num = FLOW_SENSOR_PIN,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_cfg, &pcnt_chan));
    gpio_set_pull_mode(FLOW_SENSOR_PIN, GPIO_PULLUP_ONLY);

    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

float flow_sensor_read(void)
{
    int ticks = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &ticks));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    return (float)ticks / FLOW_CALIBRATION_FACTOR; // L/s
}

// The level sensor is a potentiometer between the sense line and ground. It
// creates a resistance between 0 and 100 Ohms.
// On the r1 board the sense line is connected to +5V through a 220 Ohms 
// resistor. This produces the following voltages:
// Tank full (0 Ohms): 0V
// Tank empty (100 Ohms): 5 * 100 / (100+330) = 1.16V
// To bring it into range for the ADC when choosing the right attenuation
// (ADC_ATTEN_DB_2_5).
//
// The ADC returns a value in [0,4096). This is then converted into a value in
// liters by mapping that range to [LEVEL_CALIBRATION_LOW, LEVEL_CALIBRATION_HIGH).
// These can be set to whatever, like 0 -> 100% or 0 -> 30 Liters.
void level_sensor_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT); // 12 bits
    adc1_config_channel_atten(LEVEL_SENSOR_PIN, ADC_ATTEN_DB_2_5); // 0-1250 mV
}

float level_sensor_read(void)
{
    const float level_raw = (float) adc1_get_raw(LEVEL_SENSOR_PIN); // 0..4096
    // lerp
    const float level = LEVEL_CALIBRATION_LOW +
        ((level_raw * (LEVEL_CALIBRATION_HIGH - LEVEL_CALIBRATION_LOW)) / 4096.0);

    return level;
}

void task_sensors(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    flow_sensor_init();
    level_sensor_init();

    while (1) {
        sensor_readings_t readings;
        readings.flow_lph = flow_sensor_read() * 3600.0; // L/s -> L/h
        readings.level = level_sensor_read();

        xQueueSend(readings_queue, &readings, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    };
}

void task_display(void *pvParameters)
{
    sensor_readings_t readings;

    while (1) {
        if (xQueueReceive(readings_queue, &readings, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Flow: %f L/h\tLevel: %f", readings.flow_lph, readings.level);
        }
    }
}

void app_main(void)
{
    readings_queue = xQueueCreate(3, sizeof(sensor_readings_t));

    xTaskCreate(
            task_sensors,
            "sensors",
            4096,
            NULL,
            5,
            NULL
    );

    xTaskCreate(
            task_display,
            "display",
            4096,
            NULL,
            4,
            NULL
    );
}
