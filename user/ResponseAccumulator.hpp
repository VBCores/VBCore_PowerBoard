#pragma once

#include "main.h"

template<size_t MAX_SIZE=256>
class UARTResponseAccumulator {
private:
    char buffer[MAX_SIZE];
    size_t pos;

public:
    UARTResponseAccumulator() : pos(0) {}

    ~UARTResponseAccumulator() {
        if (pos > 0) {
            UART2_printf("%.*s\n\r", pos, buffer);
        }
    }

    void append(const char* fmt, auto... args) {
        if (pos >= MAX_SIZE) return;
        int written = snprintf(buffer + pos, MAX_SIZE - pos, fmt, args...);
        if (written > 0) {
            pos += std::min(written, static_cast<int>(MAX_SIZE - pos));
        }
    }
};
