#include "main.h"

#include <string>

#include "battery_config.h"
#include "communication.h"

#include <voltbro/utils.hpp>

ReservedObject<FlashStorage> storage;
ConfigData config_data;

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

void process_command(std::string command) {
#ifdef DEBUG
    UART2_printf("Received: <%s> \n\r", command.c_str());
    //UART2_printf("Very long receive confirmation, definitely longer than 32 bytes! \n\r");
#endif
}
