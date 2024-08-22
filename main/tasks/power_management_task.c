#include "EMC2302.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_state.h"
#include "math.h"
#include "mining.h"
#include "nvs_config.h"
#include "serial.h"
#include "TMP1075.h"
#include "TPS546.h"
#include "vcore.h"
#include <string.h>

#define POLL_RATE 2000
#define MAX_TEMP 90.0
#define THROTTLE_TEMP 75.0
#define THROTTLE_TEMP_RANGE (MAX_TEMP - THROTTLE_TEMP)

#define VOLTAGE_START_THROTTLE 4900
#define VOLTAGE_MIN_THROTTLE 3500
#define VOLTAGE_RANGE (VOLTAGE_START_THROTTLE - VOLTAGE_MIN_THROTTLE)

#define TPS546_THROTTLE_TEMP 105.0
#define TPS546_MAX_TEMP 145.0

static const char * TAG = "power_management";

static float _fbound(float value, float lower_bound, float upper_bound)
{
    if (value < lower_bound)
        return lower_bound;
    if (value > upper_bound)
        return upper_bound;

    return value;
}

static uint16_t max(uint16_t a, uint16_t b) {
    return (a > b) ? a : b;
}

// Set the fan speed between 20% min and 100% max based on chip temperature as input.
// The fan speed increases from 20% to 100% proportionally to the temperature increase from 50 and THROTTLE_TEMP
static double automatic_fan_speed(float chip_temp, GlobalState * GLOBAL_STATE)
{
    double result = 0.0;
    double min_temp = 45.0;
    double min_fan_speed = 35.0;

    if (chip_temp < min_temp) {
        result = min_fan_speed;
    } else if (chip_temp >= THROTTLE_TEMP) {
        result = 100;
    } else {
        double temp_range = THROTTLE_TEMP - min_temp;
        double fan_range = 100 - min_fan_speed;
        result = ((chip_temp - min_temp) / temp_range) * fan_range + min_fan_speed;
    }

    float perc = (float) result / 100;
    switch (GLOBAL_STATE->device_model) {
        case DEVICE_HEX:
            EMC2302_set_fan_speed(0, perc);
            EMC2302_set_fan_speed(1, perc);
            break;
        default:
    }
	return result;
}

void POWER_MANAGEMENT_task(void * pvParameters)
{
    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    PowerManagementModule * power_management = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;

    power_management->frequency_multiplier = 1;

    int last_frequency_increase = 0;

    uint16_t frequency_target = nvs_config_get_u16(NVS_CONFIG_ASIC_FREQ, CONFIG_ASIC_FREQUENCY);

    uint16_t auto_fan_speed = nvs_config_get_u16(NVS_CONFIG_AUTO_FAN_SPEED, 1);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    while (1) {
        // get voltage, current, power, fan_rpm
        switch (GLOBAL_STATE->device_model) {
            case DEVICE_HEX:
                power_management->voltage = TPS546_get_vin() * 1000;
                power_management->current = TPS546_get_iout() * 1000;

                // calculate regulator power (in milliwatts)
                power_management->power = (TPS546_get_vout() * power_management->current) / 1000;

                // get the fan RPM
                power_management->fan_rpm = max(EMC2302_get_fan_speed(0), EMC2302_get_fan_speed(1));
                break;
            default:
        }

        // get chip_temp_avg, vr_temp
        // set overheat mode
        switch (GLOBAL_STATE->device_model) {
            case DEVICE_HEX:        // Two board temperature sensors
                power_management->board_temp_1 = TMP1075_read_temperature(0);
                power_management->board_temp_2 = TMP1075_read_temperature(1);

                // ESP_LOGI(TAG, "Board Temp: %f, %f", power_management->board_temp_1, power_management->board_temp_2);

                // get regulator internal temperature
                power_management->chip_temp_avg = (TMP1075_read_temperature(0) + TMP1075_read_temperature(1)) / 2 + 10; // estimate the ASIC temperature for Hex
                power_management->vr_temp = (float)TPS546_get_temperature();

                // TODO figure out best way to detect overheating on the Hex
                if ((power_management->vr_temp > TPS546_THROTTLE_TEMP || power_management->chip_temp_avg > THROTTLE_TEMP) &&
                    (power_management->frequency_value > 50 || power_management->voltage > 1000)) {
                    ESP_LOGE(TAG, "OVERHEAT  VR: %fC ASIC %fC", power_management->vr_temp, power_management->chip_temp_avg );

                    EMC2302_set_fan_speed(0, 1);
                    EMC2302_set_fan_speed(1, 1);

                    // Turn off core voltage
                    VCORE_set_voltage(0.0, GLOBAL_STATE);

                    nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, 1000);
                    nvs_config_set_u16(NVS_CONFIG_ASIC_FREQ, 50);
                    nvs_config_set_u16(NVS_CONFIG_FAN_SPEED, 100);
                    nvs_config_set_u16(NVS_CONFIG_AUTO_FAN_SPEED, 0);
                    nvs_config_set_u16(NVS_CONFIG_OVERHEAT_MODE, 1);
                    exit(EXIT_FAILURE);
                }

                ESP_LOGI(TAG, "VIN: %f, VOUT: %f, IOUT: %f", TPS546_get_vin(), TPS546_get_vout(), TPS546_get_iout());
                // ESP_LOGI(TAG, "Regulator power: %f mW", power_management->power);
                // ESP_LOGI(TAG, "TPS546 Temp: %2f", power_management->vr_temp);
                // ESP_LOGI(TAG, "TPS546 Frequency %d", TPS546_get_frequency());
                break;
            default:
        }

        if (auto_fan_speed == 1) {

            power_management->fan_perc = (float)automatic_fan_speed(power_management->chip_temp_avg, GLOBAL_STATE);

        } else {         
            switch (GLOBAL_STATE->device_model) {
                case DEVICE_HEX:
                    float fs = (float) nvs_config_get_u16(NVS_CONFIG_FAN_SPEED, 100);
                    power_management->fan_perc = fs;
                    EMC2302_set_fan_speed(0, (float) fs / 100);
                    EMC2302_set_fan_speed(1, (float) fs / 100);
                    break;
                default:
            }
        }

        vTaskDelay(POLL_RATE / portTICK_PERIOD_MS);
    }
}

