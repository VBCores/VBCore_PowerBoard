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

        config_data.node_id = 9;
        config_data.speed_preset = CANSpeedPreset::NOM1000_DATA8000;
        config_data.auto_disarm = false;

        UART2_printf("Saving config \r\n");
        save_config();
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
static constexpr std::string SPEED_PRESET_PARAM = "speed_preset";
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
    std::string response_buffer;  // Buffer to accumulate responses
    auto append_response = [&](const char* fmt, auto... args) {
        int size = snprintf(nullptr, 0, fmt, args...);
        if (size <= 0) return;

        size_t prev_size = response_buffer.size();
        response_buffer.resize(prev_size + size);
        snprintf(&response_buffer[prev_size], size + 1, fmt, args...);
    };

    command.erase(command.find_last_not_of(" \t\n\r") + 1);
    if (command.size() == 0) {
        return;
    }

    if (command == "START") {
        enable_config_mode();
        append_response("CONFIG MODE ENABLED\n\r");

        append_response(
            "UVLO=%4.1f HYST=%4.1f FULL_CHRG=%4.1f CHRG_CURR=%4.1f\n\r",
            get_uvlo_level(),
            get_uvlo_hyst(),
            get_src_charged_level(),
            get_nom_chrg_curr()
        );
        append_response("node_id:%d\n\r", config_data.node_id);
        append_response("speed_preset:%d\n\r", to_underlying(config_data.speed_preset));
        append_response("auto_disarm:%d\n\r", config_data.auto_disarm);

        goto send;
    }
    else if (command == "APPLY") {
        NVIC_SystemReset();
    }
    else if (command == "STOP") {
        disable_config_mode();
        append_response("NOTE: config changes not applied! To apply, run APPLY or reset controller\n\r");
        goto send;
    }

    // Если не в режиме конфигурации, игнорируем команды (кроме START)
    if (app_state != AppState::CONFIG) {
        goto send;
    }

    // Обработка запроса параметра (формат "param_name:?")
    {
        size_t colon_pos = command.find(':');
        if (colon_pos == std::string::npos) {
            append_response("ERROR: Unknown command\n\r");
            goto send;
        }

        std::string param = command.substr(0, colon_pos);
        std::string value = command.substr(colon_pos + 1);

        if (value == "?") {
            // GET
            if (param == NODE_ID_PARAM) {
                append_response("node_id:%d\n\r", config_data.node_id);
            }
            else if (param == SPEED_PRESET_PARAM) {
                append_response("speed_preset:%d\n\r", to_underlying(config_data.speed_preset));
            }
            else if (param == AUTO_DISARM_PARAM) {
                append_response("auto_disarm:%d\n\r", config_data.auto_disarm);
            }
            else if (param == UVLO_LEVEL_PARAM) {
                append_response("uvlo_level:%4.2f\n\r", config_data.uvlo_level);
            }
            else if (param == UVLO_HYST_PARAM) {
                append_response("uvlo_hyst:%4.2f\n\r", config_data.uvlo_hyst);
            }
            else if (param == NOMINAL_CHARGE_PARAM) {
                append_response("nominal_charge_current:%4.2f\n\r", config_data.nominal_charge_current);
            }
            else if (param == CHARGED_LEVEL_PARAM) {
                append_response("charged_level:%4.2f\n\r", config_data.src_charged_level);
            }
            else {
                append_response("ERROR: Unknown parameter\n\r");
            }
        }
        else {
            bool do_save = true;
            // SET

            int new_int_value;
            float new_float_value;
            bool is_converted = false;
            if (param == NODE_ID_PARAM ||
                param == SPEED_PRESET_PARAM ||
                param == AUTO_DISARM_PARAM) {
                is_converted = safe_stoi(value, new_int_value);
            }
            else {
                is_converted = safe_stof(value, new_float_value);
            }
            if (!is_converted) {
                append_response("ERROR: Invalid value\n\r");
                goto send;
            }

            if (param == NODE_ID_PARAM) {
                config_data.node_id = static_cast<CanardNodeID>(new_int_value);
                append_response("OK: node_id:%d\n\r", config_data.node_id);
            }
            else if (param == SPEED_PRESET_PARAM) {
                config_data.speed_preset = static_cast<CANSpeedPreset>(new_int_value);
                append_response("OK: speed_preset:%d\n\r", config_data.speed_preset);
            }
            else if (param == AUTO_DISARM_PARAM) {
                config_data.auto_disarm = static_cast<bool>(new_int_value);
                append_response("OK: auto_disarm:%d\n\r", config_data.auto_disarm);
            }
            else if (param == UVLO_LEVEL_PARAM) {
                config_data.uvlo_level = new_float_value;
                append_response("OK: uvlo_level:%4.2f\n\r", config_data.uvlo_level);
            }
            else if (param == UVLO_HYST_PARAM) {
                config_data.uvlo_hyst = new_float_value;
                append_response("OK: uvlo_hyst:%4.2f\n\r", config_data.uvlo_hyst);
            }
            else if (param == CHARGED_LEVEL_PARAM) {
                config_data.src_charged_level = new_float_value;
                append_response("OK: charged_level:%4.2f\n\r", config_data.src_charged_level);
            }
            else if (param == NOMINAL_CHARGE_PARAM) {
                config_data.nominal_charge_current = new_float_value;
                append_response("OK: nominal_charge_current:%4.2f\n\r", config_data.nominal_charge_current);
            }
            else {
                append_response("ERROR: Unknown parameter\n\r");
                do_save = false;
            }

            if (do_save) {
                save_config();
                append_response("Saved config to FLASH\n\r");
            }
        }
    }

send:
    if (!response_buffer.empty()) {
        UART2_printf("%s", response_buffer.c_str());
    }
}
