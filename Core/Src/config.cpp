#include "main.h"

#include "flash.hpp"
#include "battery_config.h"

#include <voltbro/utils.hpp>

ReservedObject<FlashStorage> storage;

struct alignas(8) ConfigData {
    //static constexpr uint32_t TYPE_ID = 0x12345678;
    static constexpr uint32_t TYPE_ID = 0x01234567;
    uint32_t type_id;              // 4 bytes

    float uvlo_level;              // 4 bytes
    float uvlo_hyst;               // 4 bytes
    float src_charged_level;       // 4 bytes
    float nominal_charge_current;  // 4 bytes

    uint8_t _padding[4];           // 4 bytes
};
static_assert(sizeof(ConfigData) % 8 == 0, "Size must be multiple of 8 bytes for FLASH operations");
static ConfigData config_data;

float get_uvlo_level() {
    return config_data.uvlo_level;
}

float get_uvlo_hyst() {
    return config_data.uvlo_hyst;
}

float get_src_charged_level() {
    return config_data.src_charged_level;
}

float get_nom_chrg_curr() {
    return config_data.nominal_charge_current;
}

void save_config() {
    UART2_printf("Saving config \r\n");
    auto status = storage->write(&config_data, 0);
    if (status != HAL_OK) {
        Error_Handler();
    }
}

void load_config() {
    storage.create(&hiwdg);

    auto status = storage->read<ConfigData>(&config_data, 0);
    if (status != HAL_OK) {
        Error_Handler();
    }
    UART2_printf("Got config_data type_id: <0x%08lX> \r\n", config_data.type_id);

    if (config_data.type_id != ConfigData::TYPE_ID) {
        // initialize with default values
        config_data.type_id = ConfigData::TYPE_ID;
        config_data.uvlo_level = DEFAULT_UVLO_LEVEL;
        config_data.uvlo_hyst = DEFAULT_UVLO_HYST;
        config_data.src_charged_level = DEFAULT_SRC_CHARGED_LEVEL;
        config_data.nominal_charge_current = DEFAULT_NOMINAL_CHARGE_CURRENT;

        save_config();
    }
}
