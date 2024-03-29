#include "main.h"
#include "hmi.h"

#include <uavcan/node/Heartbeat_1_0.h>

#include "voltbro/utils.hpp"

#include "cyphal/cyphal.h"
#include <cyphal/allocators/o1/o1_allocator.h>
#include <cyphal/providers/G4CAN.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/node/node_info_handler.h>

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

extern FDCAN_HandleTypeDef hfdcan1;

static constexpr CanardNodeID NODE_ID = 9;
static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;

bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;
UtilityConfig utilities(micros_64, Error_Handler);

ReservedObject<NodeInfoReader> node_info_reader;

bool was_initialized = false;

void set_mode(uint8_t mode) {
    CYPHAL_MODE = mode;
}

static uint32_t uptime = 0;
void heartbeat() {
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
        .uptime = uptime,
        .health = {CYPHAL_HEALTH_STATUS},
        .mode = {CYPHAL_MODE}
    };
    cyphal_interface->send_msg<HBeat>(
        &heartbeat_msg,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        &hbeat_transfer_id,
        MICROS_S * 2
    );
}

void user_setup(void) {
    cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        NODE_ID,
        &hfdcan1,
        200,
        utilities
    ));
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.power_board",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    static FDCAN_FilterTypeDef sFilterConfig;

    HAL_IMPORTANT(apply_filter(
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(NODE_ID)
    ))

    HAL_IMPORTANT(HAL_FDCAN_ConfigTxDelayCompensation(
        &hfdcan1,
        hfdcan1.Init.DataTimeSeg1 * hfdcan1.Init.DataPrescaler,
        0
    ))
    HAL_IMPORTANT(HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1))

    HAL_IMPORTANT(HAL_FDCAN_Start(&hfdcan1))

    _is_cyphal_on = true;
    set_mode(uavcan_node_Mode_1_0_OPERATIONAL);
}

static millis battery_report_time = 0;
static millis heartbeat_time = 0;
void reporting_loop(millis millis) {
    EACH_N(millis, heartbeat_time, 1000, {
        uptime++;
        heartbeat();
    })
}

void user_spin(void) {
    reporting_loop(HAL_GetTick());
}
