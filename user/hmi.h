#include <cstdint>

enum class HMI_LED{
    FIRST,
    SECOND
};

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void start_led(HMI_LED target, RGB color, int beeps, float freq);
void start_beeper(int beeps, float freq);
void hmi_handler(uint64_t cur_micros);
