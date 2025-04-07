#include "main.h"

#include <string>

#include "battery_config.h"
#include "communication.h"
#include "ResponseAccumulator.hpp"

#include <voltbro/utils.hpp>

ReservedObject<FlashStorage> storage;
ConfigData config_data;

uint8_t get_nom_prescaler() {
    switch (config_data.fdcan_nominal_baud) {
        case FDCANNominalBaud::KHz62:
            return 16;
        case FDCANNominalBaud::KHz125:
            return 8;
        case FDCANNominalBaud::KHz250:
            return 4;
        case FDCANNominalBaud::KHz500:
            return 2;
        case FDCANNominalBaud::KHz1000:
            return 1;
        default:
            Error_Handler();
    }
}

uint8_t get_data_prescaler() {
    switch (config_data.fdcan_data_baud) {
        case FDCANDataBaud::KHz1000:
            return 8;
        case FDCANDataBaud::KHz2000:
            return 4;
        case FDCANDataBaud::KHz4000:
            return 2;
        case FDCANDataBaud::KHz8000:
            return 1;
        default:
            Error_Handler();
    }
}

bool is_bus_off_while_charging() {
    return !config_data.auto_disarm;
}

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
    auto status = storage->write(&config_data, 0);
    if (status != HAL_OK) {
        Error_Handler();
    }
}

void set_default_config() {
    config_data.type_id = ConfigData::TYPE_ID;
    config_data.was_configured = false;

    config_data.uvlo_level = NAN;
    config_data.uvlo_hyst = NAN;
    config_data.src_charged_level = NAN;
    config_data.nominal_charge_current = NAN;

    config_data.node_id = 9;
    config_data.fdcan_data_baud = FDCANDataBaud::KHz8000;
    config_data.fdcan_nominal_baud = FDCANNominalBaud::KHz1000;
    config_data.auto_disarm = true;
}

void load_config() {
    UARTResponseAccumulator responses;

    storage.create(&hiwdg);

    auto status = storage->read<ConfigData>(&config_data, 0);
    if (status != HAL_OK) {
        Error_Handler();
    }
    responses.append("Got config_data type_id: <0x%08lX>\r\n", config_data.type_id);

    if (config_data.type_id != ConfigData::TYPE_ID) {
        set_default_config();
        responses.append("Saving config...\r\n");
        save_config();
    }

    if (!config_data.was_configured) {
        set_bus_state(false);
        set_pc_state(false);
        responses.append("Board was not configured!\r\n");
        HAL_TIM_Base_Start_IT(&htim1);
    }
    else {
        set_bus_state(true);
        set_pc_state(true);
        responses.append("Board is configured, starting\r\n");
        responses.append(
            "uvlo_level=%4.1f uvlo_hyst=%4.1f charged_level=%4.1f charge_current=%4.1f\n\r",
            config_data.uvlo_level,
            config_data.uvlo_hyst,
            config_data.src_charged_level,
            config_data.nominal_charge_current
        );
        HAL_TIM_Base_Stop_IT(&htim1);
    }
}

bool safe_stoi(const std::string& str, int& out_val) {
    if (str.empty()) return false;

    char* endptr = nullptr;
    long val = strtol(str.c_str(), &endptr, 10);

    // Check for conversion errors
    if (endptr != str.c_str() + str.size()) return false;
    if (val < INT32_MIN || val > INT32_MAX) return false;

    out_val = static_cast<int>(val);
    return true;
}

bool safe_stof(const std::string& str, float& out_val) {
    if (str.empty()) return false;

    char* endptr = nullptr;
    float val = strtof(str.c_str(), &endptr);

    if (endptr != str.c_str() + str.size()) return false;
    if (val == HUGE_VALF || val == -HUGE_VALF) return false;

    out_val = val;
    return true;
}

static constexpr std::string NODE_ID_PARAM = "node_id";
static constexpr std::string FDCAN_DATA_PARAM = "data_baud";
static constexpr std::string FDCAN_NOMINAL_PARAM = "nominal_baud";
static constexpr std::string AUTO_DISARM_PARAM = "auto_disarm";
static constexpr std::string UVLO_LEVEL_PARAM = "uvlo_level";
static constexpr std::string UVLO_HYST_PARAM = "uvlo_hyst";
static constexpr std::string NOMINAL_CHARGE_PARAM = "charge_current";
static constexpr std::string CHARGED_LEVEL_PARAM = "charged_level";

enum class AppState {
    RUNNING,
    CONFIG
};

static AppState app_state = AppState::RUNNING;

void enable_config_mode() {
    app_state = AppState::CONFIG;
    set_bus_state(false);
    set_pc_state(false);
}

void disable_config_mode() {
    app_state = AppState::RUNNING;
    set_bus_state(true);
    set_pc_state(true);
}


void process_command(std::string command) {
    UARTResponseAccumulator responses;

    command.erase(command.find_last_not_of(" \t\n\r") + 1);
    if (command.size() == 0) {
        return;
    }

    bool do_save = false;

    if (command == "START") {
        enable_config_mode();
        responses.append("CONFIG MODE ENABLED\n\r");

        responses.append(
            "uvlo_level=%4.1f uvlo_hyst=%4.1f charged_level=%4.1f charge_current=%4.1f\n\r",
            config_data.uvlo_level,
            config_data.uvlo_hyst,
            config_data.src_charged_level,
            config_data.nominal_charge_current
        );
        responses.append("node_id:%d\n\r", config_data.node_id);
        responses.append("data_baud:%d\n\r", to_underlying(config_data.fdcan_data_baud));
        responses.append("nominal_baud:%d\n\r", to_underlying(config_data.fdcan_nominal_baud));
        responses.append("auto_disarm:%d\n\r", config_data.auto_disarm);

        return;
    }
    else if (command == "APPLY") {
        NVIC_SystemReset();
    }
    else if (command == "RESET") {
        responses.append("Setting default config\n\r");
        set_default_config();
        responses.append("NOTE: config changes not applied! To apply, run APPLY or reset controller\n\r");
        do_save = true;
    }
    else if (command == "STOP") {
        disable_config_mode();
        responses.append("NOTE: config changes not applied! To apply, run APPLY or reset controller\n\r");
        return;
    }
    else {
        // Если не в режиме конфигурации, игнорируем команды (кроме START)
        if (app_state != AppState::CONFIG) {
            return;
        }

        // Обработка запроса параметра (формат "param_name:?")
        size_t colon_pos = command.find(':');
        if (colon_pos == std::string::npos) {
            responses.append("ERROR: Unknown command\n\r");
            return;
        }

        std::string param = command.substr(0, colon_pos);
        std::string value = command.substr(colon_pos + 1);

        if (value == "?") {
            // GET
            if (param == NODE_ID_PARAM) {
                responses.append("node_id:%d\n\r", config_data.node_id);
            }
            else if (param == FDCAN_DATA_PARAM) {
                responses.append("data_baud:%d\n\r", to_underlying(config_data.fdcan_data_baud));
            }
            else if (param == FDCAN_NOMINAL_PARAM) {
                responses.append("nominal_baud:%d\n\r", to_underlying(config_data.fdcan_nominal_baud));
            }
            else if (param == AUTO_DISARM_PARAM) {
                responses.append("auto_disarm:%d\n\r", config_data.auto_disarm);
            }
            else if (param == UVLO_LEVEL_PARAM) {
                responses.append("uvlo_level:%4.2f\n\r", config_data.uvlo_level);
            }
            else if (param == UVLO_HYST_PARAM) {
                responses.append("uvlo_hyst:%4.2f\n\r", config_data.uvlo_hyst);
            }
            else if (param == NOMINAL_CHARGE_PARAM) {
                responses.append("nominal_charge_current:%4.2f\n\r", config_data.nominal_charge_current);
            }
            else if (param == CHARGED_LEVEL_PARAM) {
                responses.append("charged_level:%4.2f\n\r", config_data.src_charged_level);
            }
            else {
                responses.append("ERROR: Unknown parameter\n\r");
            }
        }
        else {
            // SET
            do_save = true;
            int new_int_value;
            float new_float_value;
            bool is_converted = false;
            if (param == NODE_ID_PARAM ||
                param == FDCAN_DATA_PARAM ||
                param == FDCAN_NOMINAL_PARAM ||
                param == AUTO_DISARM_PARAM) {
                is_converted = safe_stoi(value, new_int_value);
            }
            else {
                is_converted = safe_stof(value, new_float_value);
            }
            if (!is_converted) {
                responses.append("ERROR: Invalid value\n\r");
                return;
            }

            if (param == NODE_ID_PARAM) {
                config_data.node_id = static_cast<CanardNodeID>(new_int_value);
                responses.append("OK: node_id:%d\n\r", config_data.node_id);
            }
            else if (param == FDCAN_DATA_PARAM) {
                config_data.fdcan_data_baud = static_cast<FDCANDataBaud>(new_int_value);
                responses.append("OK: data_baud:%d\n\r", config_data.fdcan_data_baud);
            }
            else if (param == FDCAN_NOMINAL_PARAM) {
                config_data.fdcan_nominal_baud = static_cast<FDCANNominalBaud>(new_int_value);
                responses.append("OK: nominal_baud:%d\n\r", config_data.fdcan_nominal_baud);
            }
            else if (param == AUTO_DISARM_PARAM) {
                config_data.auto_disarm = static_cast<bool>(new_int_value);
                responses.append("OK: auto_disarm:%d\n\r", config_data.auto_disarm);
            }
            else if (param == UVLO_LEVEL_PARAM) {
                config_data.uvlo_level = new_float_value;
                responses.append("OK: uvlo_level:%4.2f\n\r", config_data.uvlo_level);
            }
            else if (param == UVLO_HYST_PARAM) {
                config_data.uvlo_hyst = new_float_value;
                responses.append("OK: uvlo_hyst:%4.2f\n\r", config_data.uvlo_hyst);
            }
            else if (param == CHARGED_LEVEL_PARAM) {
                config_data.src_charged_level = new_float_value;
                responses.append("OK: charged_level:%4.2f\n\r", config_data.src_charged_level);
            }
            else if (param == NOMINAL_CHARGE_PARAM) {
                config_data.nominal_charge_current = new_float_value;
                responses.append("OK: nominal_charge_current:%4.2f\n\r", config_data.nominal_charge_current);
            }
            else {
                responses.append("ERROR: Unknown parameter\n\r");
                do_save = false;
            }
        }
    }

    if (do_save) {
        if (
            !std::isnan(config_data.uvlo_hyst) &&
            !std::isnan(config_data.uvlo_level) &&
            !std::isnan(config_data.src_charged_level) &&
            !std::isnan(config_data.nominal_charge_current)
        ) {
            config_data.was_configured = true;
            responses.append("All essential parameters set, board will start after APPLY\n\r");
        }

        save_config();
        responses.append("Saved config to FLASH\n\r");
    }
}
