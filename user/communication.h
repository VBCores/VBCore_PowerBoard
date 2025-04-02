#pragma once

#include <string_view>

#include "flash.hpp"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>

enum class CANSpeedPreset : uint8_t {
    NOM1000_DATA8000
};

struct alignas(8) ConfigData {
    static constexpr uint32_t TYPE_ID = 0x01234567;
    uint32_t type_id;              // 4 bytes

    CanardNodeID node_id;          // 1 byte
    CANSpeedPreset speed_preset;   // 1 byte
    bool auto_disarm;  // 1 byte
    float uvlo_level;              // 4 bytes
    float uvlo_hyst;               // 4 bytes
    float src_charged_level;       // 4 bytes
    float nominal_charge_current;  // 4 bytes

    uint8_t _padding[1];           // 1 bytes
};
static_assert(sizeof(ConfigData) % 8 == 0, "Size must be multiple of 8 bytes for FLASH operations");

extern ReservedObject<FlashStorage> storage;
extern ConfigData config_data;

void save_config();
void start_uart_recv_it(UART_HandleTypeDef*);
void process_command(std::string command);
