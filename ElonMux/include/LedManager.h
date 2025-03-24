#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <Arduino.h>
#include "driver/ledc.h"

class LedManager {
public:
    LedManager();
    void begin();

    void LEDON();
    void LEDOFF();

    void fadeIn(uint32_t durationMs);
    void fadeOut(uint32_t durationMs);

    // Existing indefinite blink
    void startBlink(uint32_t onTimeMs, uint32_t offTimeMs);
    // Make the LED fade in and out indefinitely
    void startFading(uint32_t durationMs);

    /**
     * NEW: Start a FreeRTOS task to blink the LED for a limited duration,
     * rather than indefinitely. By default, blinks for 3000 ms (3 seconds).
     */
    void startBlinkForDuration(uint32_t onTimeMs, uint32_t offTimeMs, uint32_t totalDurationMs = 3000);

private:
    // Current duty
    volatile int duty_;

    // Task handles
    TaskHandle_t fadeInTaskHandle_;
    TaskHandle_t fadeOutTaskHandle_;
    TaskHandle_t blinkTaskHandle_;
    TaskHandle_t fadeInOutTaskHandle_;

    void configLedc();
    static void fadeInTask(void* parameter);
    static void fadeOutTask(void* parameter);
    static void fadeInOutTask(void* parameter);
    static void blinkTask(void* parameter);

    // NEW: Task entry point for blink-with-duration
    static void blinkWithDurationTask(void* parameter);

    void killAllTasks();

    // LEDC channel parameters
    static constexpr int LED_PIN          = 21;
    static constexpr ledc_mode_t LEDC_MODE   = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
    static constexpr ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;
    static constexpr int LEDC_FREQUENCY   = 5000;
    static constexpr ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_10_BIT;
    static constexpr int LEDC_MAX_DUTY    = 1023;
};

// NEW struct for timed blinking
struct BlinkWithDurationTaskParams {
    LedManager* ledMgr;
    uint32_t onTimeMs;
    uint32_t offTimeMs;
    uint32_t totalDurationMs;
};

#endif // LED_MANAGER_H
