#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <algorithm>
#include "stm32g4xx_hal.h"

#include "main.h"

class FlashStorage {
private:
    IWDG_HandleTypeDef* _hiwdg;
    // Device-specific constants from datasheet
    static constexpr uint32_t PAGES_PER_BANK = 128;            // G4 series has 128 pages/bank
    static constexpr uint32_t TOTAL_FLASH_SIZE = 512 * 1024;   // 512KB for G474RET3

    uint32_t _start;
    uint32_t _size;

    // Memory barriers and critical section helpers
    template<typename F>
    HAL_StatusTypeDef __attribute__((section(".RamFunc"))) execute_flash_operation(F&& f) const {
        __disable_irq();
        __DSB();
        __ISB();

        const auto status = f();
        HAL_IWDG_Refresh(_hiwdg);

        __DSB();
        __ISB();
        __enable_irq();
        return status;
    }

    uint32_t get_bank(uint32_t page_number) const {
        return (page_number < PAGES_PER_BANK) ? FLASH_BANK_1 : FLASH_BANK_2;
    }

    uint32_t get_page_number(uint32_t address) const {
        return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    }

    HAL_StatusTypeDef validate_offset(uint32_t offset, uint32_t data_size) const {
        const bool valid = (
            (offset % sizeof(uint64_t) == 0) &&
            (data_size <= _size) &&
            (offset <= (_size - data_size))
        );
        return valid ? HAL_OK : HAL_ERROR;
    }

    HAL_StatusTypeDef erase_pages(uint32_t first_page, uint32_t last_page) {

        FLASH_EraseInitTypeDef erase_init = {
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .Banks = get_bank(first_page),
            .Page = first_page % PAGES_PER_BANK,  // Bank-relative page index
            .NbPages = last_page - first_page + 1
        };

        uint32_t page_error;

        auto status =  HAL_FLASHEx_Erase(&erase_init, &page_error);
        HAL_IWDG_Refresh(_hiwdg);

        return status;
    }

    HAL_StatusTypeDef verify_erasure(uint32_t page) const {
        const uint32_t page_addr = FLASH_BASE + (page * FLASH_PAGE_SIZE);
        const uint64_t* addr = reinterpret_cast<const uint64_t*>(page_addr);

        // Check first 8 double words (64 bytes) as erased state
        for (uint32_t i = 0; i < 8; ++i) {
            if (addr[i] != 0xFFFFFFFFFFFFFFFF) return HAL_ERROR;
        }
        return HAL_OK;
    }

    HAL_StatusTypeDef programm_data(uint32_t address, const void* data, size_t size) {
        const uint64_t* src = reinterpret_cast<const uint64_t*>(data);
        const size_t num_words = size / sizeof(uint64_t);

        HAL_StatusTypeDef status = HAL_OK;
        for (size_t i = 0; i < num_words; ++i) {
            status = HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_DOUBLEWORD,
                address + (i * sizeof(uint64_t)),
                src[i]
            );
            if (status != HAL_OK) break;
        }
        return status;
    }

    HAL_StatusTypeDef verify_programming(uint32_t address, const void* data, size_t size) const {
        const uint64_t* expected = reinterpret_cast<const uint64_t*>(data);
        const uint64_t* actual = reinterpret_cast<const uint64_t*>(address);
        const size_t num_words = size / sizeof(uint64_t);

        for (size_t i = 0; i < num_words; ++i) {
            if (actual[i] != expected[i]) {
                return HAL_ERROR;
            }
        }
        return HAL_OK;
    }

public:
    // Uses last page by default (adjust according to your memory layout)
    FlashStorage(IWDG_HandleTypeDef* hiwdg)
        : _hiwdg(hiwdg),
          _start(FLASH_BASE + TOTAL_FLASH_SIZE - FLASH_PAGE_SIZE),
          _size(FLASH_PAGE_SIZE) {}

    HAL_StatusTypeDef write(const void* data, uint32_t offset, size_t size) {
        // Check alignment and size constraints
        if (reinterpret_cast<uintptr_t>(data) % alignof(uint64_t) != 0) {
            return HAL_ERROR;
        }

        auto status = validate_offset(offset, size);
        if (status != HAL_OK) return status;

        const uint32_t address = _start + offset;
        const uint32_t first_page = get_page_number(address);
        const uint32_t last_page = get_page_number(address + size - 1);

        // Check if pages need erasing
        bool needs_erase = false;
        for (uint32_t page = first_page; page <= last_page; ++page) {
            if (verify_erasure(page) != HAL_OK) {
                needs_erase = true;
                break;
            }
        }

        // Delaying watchdog period - there is not way to disable IWDG after start
        auto backup_iwdg_prescaler = _hiwdg->Instance->PR;
        auto backup_iwdg_reload = _hiwdg->Instance->RLR;
        _hiwdg->Instance->KR = 0x5555;
        _hiwdg->Instance->PR = IWDG_PRESCALER_256;
        _hiwdg->Instance->RLR = 0xFFF;
        while (IWDG->SR != 0) {  // waiting for timing settings to apply
            _hiwdg->Instance->KR = 0xAAAA;
        }
        HAL_Delay(5);
        status = execute_flash_operation([&]() {
            HAL_FLASH_Unlock();
            auto flash_op_st = HAL_OK;
            if (needs_erase) {
                flash_op_st = erase_pages(first_page, last_page);
                if (flash_op_st != HAL_OK) return status;
            }

            flash_op_st = programm_data(address, data, size);
            HAL_FLASH_Lock();
            return flash_op_st;
        });
        _hiwdg->Instance->KR = 0x5555;
        _hiwdg->Instance->RLR = backup_iwdg_reload;
        _hiwdg->Instance->PR = backup_iwdg_prescaler;

        if (status != HAL_OK) return status;

        return verify_programming(address, data, size);
    }

    template<typename T>
    HAL_StatusTypeDef write(const T* obj, uint32_t offset) {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        static_assert(sizeof(T) % sizeof(uint64_t) == 0, "Size must be multiple of 8 bytes");

        return write(reinterpret_cast<const void*>(obj), offset, sizeof(T));
    }

    HAL_StatusTypeDef read(void* dest, uint32_t offset, size_t size) const {
        auto status = validate_offset(offset, size);
        if (status != HAL_OK) return status;

        return execute_flash_operation([&]() {
            const uint8_t* src = reinterpret_cast<const uint8_t*>(_start + offset);
            std::memcpy(dest, src, size);
            return HAL_OK;
        });
    }

    template<typename T>
    HAL_StatusTypeDef read(T* obj, uint32_t offset) const {
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        static_assert(sizeof(T) % sizeof(uint64_t) == 0, "Size must be multiple of 8 bytes");

        return read(reinterpret_cast<void*>(obj), offset, sizeof(T));
    }
};
