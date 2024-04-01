#include "hmi.h"

#include "main.h"

extern TIM_HandleTypeDef htim3;

void setup_hmi() {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

#define SEC 1000000.0f

static uint64_t beeper_cutoff_micros = 0;
static uint64_t next_adjust_micros = 0;
static uint64_t pulse_incr = 0;
static uint64_t period_incr = 0;

void set_beeper_params(uint64_t cur_micros) {
    if (buzzer_mutex > USER) {
        return;
    }
    buzzer_pulse_stamp = cur_micros + pulse_incr;
    buzzer_period_stamp = cur_micros + period_incr;
    next_adjust_micros = buzzer_period_stamp;
}

void start_beeper(int beeps, float freq) {
    if (buzzer_mutex > USER) {
        return;
    }
    uint64_t cur_micros = micros_64();

    buzzer_mutex = USER;

    beeper_cutoff_micros = cur_micros + (uint64_t)(2 * SEC * beeps / freq);
    period_incr = (uint64_t)(SEC / freq) * 2;
    pulse_incr = period_incr / 2;

    set_beeper_params(cur_micros);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void stop_beeper() {
    if (buzzer_mutex > USER) {
        return;
    }
    buzzer_mutex = 0;
    buzzer_pulse_stamp = 0;
    buzzer_period_stamp = 0;
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

struct BlinkParams{
    uint64_t cutoff_micros;
    uint64_t toggle_micros;
    uint64_t toggle_incr;
    HMI_LED target;
};

#define LED_1_CHANNELS_COUNT 3
uint32_t led_1_channels[LED_1_CHANNELS_COUNT] = {TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
BlinkParams led_1_params = {
    .cutoff_micros = 0,
    .toggle_micros = 0,
    .toggle_incr = 0,
    .target = HMI_LED::FIRST
};

#define LED_2_CHANNELS_COUNT 3
uint32_t led_2_channels[LED_2_CHANNELS_COUNT] = {0, 0, 0};
BlinkParams led_2_params = {
    .cutoff_micros = 0,
    .toggle_micros = 0,
    .toggle_incr = 0,
    .target = HMI_LED::SECOND
};

void select_led(HMI_LED target, size_t* channels_count, uint32_t** led_channels) {
    switch (target) {
        default:
        case HMI_LED::FIRST:
            *channels_count = LED_1_CHANNELS_COUNT;
            *led_channels = led_1_channels;
            break;
        case HMI_LED::SECOND:
            *channels_count = LED_2_CHANNELS_COUNT;
            *led_channels = led_2_channels;
            break;
    }
}

void reset_led(HMI_LED target) {
    size_t channels_count;
    uint32_t* led_channels;
    select_led(target, &channels_count, &led_channels);
    for (size_t i = 0; i < channels_count; i++) {
        if (led_channels[i] == 0) continue;
        __HAL_TIM_SET_COMPARE(&htim3, led_channels[i], 0);
    }
}

void select_led_with_params(
        HMI_LED target,
        size_t* channels_count,
        uint32_t** led_channels,
        BlinkParams** params
) {
    select_led(target, channels_count, led_channels);
    switch (target) {
        default:
        case HMI_LED::FIRST:
            *params = &led_1_params;
            break;
        case HMI_LED::SECOND:
            *params = &led_2_params;
            break;
    }
}

void toggle_pwm(HMI_LED target) {
    size_t channels_count;
    uint32_t* led_channels;
    select_led(target, &channels_count, &led_channels);

    for (size_t i = 0; i < channels_count; i++) {
        if (led_channels[i] == 0) continue;
        if (TIM_CHANNEL_STATE_GET(&htim3, led_channels[i]) == HAL_TIM_CHANNEL_STATE_READY) {
            HAL_TIM_PWM_Start(&htim3, led_channels[i]);
        }
        else {
            HAL_TIM_PWM_Stop(&htim3, led_channels[i]);
        }
    }
}

void start_led(HMI_LED target, RGB color, int beeps, float freq) {
    uint64_t cur_micros = micros_64();

    size_t channels_count;
    uint32_t* led_channels;
    BlinkParams* led_params;
    select_led_with_params(target, &channels_count, &led_channels, &led_params);

    uint8_t* color_ptr = &color.r;
    for (size_t i = 0; i < channels_count; i++) {
        if (led_channels[i] == 0) continue;
        __HAL_TIM_SET_COMPARE(&htim3, led_channels[i], color_ptr[i]);
        HAL_TIM_PWM_Start(&htim3, led_channels[i]);
    }

    if (beeps <= 0) {
        led_params->cutoff_micros = 0;
        led_params->toggle_incr = 0;
        led_params->toggle_micros = 0;
        return;
    }

    uint64_t incr = (uint64_t)(SEC / freq);

    led_params->cutoff_micros = cur_micros + incr * (2 * beeps) + 1;
    led_params->toggle_micros = cur_micros + incr;
    led_params->toggle_incr = incr;
}

void stop_led(HMI_LED target) {
    reset_led(target);

    size_t channels_count;
    uint32_t* led_channels;
    select_led(target, &channels_count, &led_channels);

    for (size_t i = 0; i < channels_count; i++) {
        if (led_channels[i] == 0) continue;
        HAL_TIM_PWM_Stop(&htim3, led_channels[i]);
    }
}

void finalize_led(BlinkParams* led_params) {
    stop_led(led_params->target);
    led_params->cutoff_micros = 0;
    led_params->toggle_incr = 0;
    led_params->toggle_micros = 0;
}

void handle_led_beep(uint64_t cur_micros, BlinkParams* led_params) {
    if (led_params->cutoff_micros <= 0) return;

    if (cur_micros >= led_params->cutoff_micros) {
        finalize_led(led_params);
    }
    else if (cur_micros >= led_params->toggle_micros) {
        led_params->toggle_micros = cur_micros + led_params->toggle_incr;
        toggle_pwm(led_params->target);
    }
}

void hmi_interrupt(uint64_t cur_micros) {
    handle_led_beep(cur_micros, &led_1_params);
    handle_led_beep(cur_micros, &led_2_params);

    if (beeper_cutoff_micros > 0) {
        if (cur_micros >= beeper_cutoff_micros) {
            stop_beeper();
        }
        else if (cur_micros >= next_adjust_micros) {
            set_beeper_params(cur_micros);
        }
    }
}
