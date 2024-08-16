#include <stdio.h>
#include <math.h>
#include "esp_log.h"

#include "vcore.h"
#include "adc.h"
#include "TPS546.h"

static const char *TAG = "vcore.c";

static TPS546_CONFIG TPS546_CONFIG_HEX = {
    /* vin voltage */
    .TPS546_INIT_VIN_ON = 11.5,
    .TPS546_INIT_VIN_OFF = 11.0,
    .TPS546_INIT_VIN_UV_WARN_LIMIT = 11.0,
    .TPS546_INIT_VIN_OV_FAULT_LIMIT = 14.0,
    /* vout voltage */
    .TPS546_INIT_SCALE_LOOP = 0.125,
    .TPS546_INIT_VOUT_MIN = 2.5,
    .TPS546_INIT_VOUT_MAX = 4.5,
    .TPS546_INIT_VOUT_COMMAND = 3.6
};

void VCORE_init(GlobalState * global_state) {
    switch (global_state->device_model) {
        case DEVICE_HEX: // init TPS546 for Hex
            TPS546_init(TPS546_CONFIG_HEX);
            break;
        default:
    }

    ADC_init();
}

bool VCORE_set_voltage(float core_voltage, GlobalState * global_state)
{
    switch (global_state->device_model) {
        case DEVICE_HEX: // turn on ASIC core voltage (three domains in series)
            ESP_LOGI(TAG, "Set ASIC voltage = %.3fV", core_voltage);
            TPS546_set_vout(core_voltage * (float)global_state->voltage_domain);
            break;
        default:
    }

    return true;
}

uint16_t VCORE_get_voltage_mv(GlobalState * global_state) {
    switch (global_state->device_model) {
        case DEVICE_HEX:
            return (TPS546_get_vout() * 1000) / global_state->voltage_domain;
            break;
        default:
    }

    return ADC_get_vcore() / global_state->voltage_domain;
}
