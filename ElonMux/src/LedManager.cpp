#include "LedManager.h"

/**
 * Simple structs to pass parameters into tasks
 */
struct FadeTaskParams {
    LedManager* ledMgr;
    uint32_t durationMs;
};

struct BlinkTaskParams {
    LedManager* ledMgr;
    uint32_t onTimeMs;
    uint32_t offTimeMs;
};

// Constructor and begin
LedManager::LedManager()
    : duty_(0),
      fadeInTaskHandle_(nullptr),
      fadeOutTaskHandle_(nullptr),
      blinkTaskHandle_(nullptr)
{
}

void LedManager::begin() {
    configLedc();
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// ON/OFF methods
void LedManager::LEDON() {
    killAllTasks();
    duty_ = LEDC_MAX_DUTY;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void LedManager::LEDOFF() {
    killAllTasks();
    duty_ = 0;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Fade in/out
void LedManager::fadeIn(uint32_t durationMs) {
    if (fadeInTaskHandle_ == nullptr) {
        FadeTaskParams* params = new FadeTaskParams{ this, durationMs };
        xTaskCreate(fadeInTask, "LedManagerFadeInTask", 2048, params, 1, &fadeInTaskHandle_);
    }
}

void LedManager::fadeOut(uint32_t durationMs) {
    if (fadeOutTaskHandle_ == nullptr) {
        FadeTaskParams* params = new FadeTaskParams{ this, durationMs };
        xTaskCreate(fadeOutTask, "LedManagerFadeOutTask", 2048, params, 1, &fadeOutTaskHandle_);
    }
}

// Indefinite blink
void LedManager::startBlink(uint32_t onTimeMs, uint32_t offTimeMs) {
    killAllTasks();  // if you only want one effect running
    if (blinkTaskHandle_ == nullptr) {
        BlinkTaskParams* params = new BlinkTaskParams{ this, onTimeMs, offTimeMs };
        xTaskCreate(blinkTask, "LedManagerBlinkTask", 2048, params, 1, &blinkTaskHandle_);
    }
}

// Fade in and out indefinitely
void LedManager::startFading(uint32_t durationMs) { 
    killAllTasks();// If you only want one effect at a time:
    if (fadeInOutTaskHandle_ == nullptr) {
        FadeTaskParams* params = new FadeTaskParams{ this, durationMs };
        xTaskCreate(fadeInOutTask, "LedManagerFadeInOutTask", 2048, params, 1, &fadeInOutTaskHandle_);
    }
}

/* ------------------ NEW: Blink for a limited duration ------------------ */
void LedManager::startBlinkForDuration(uint32_t onTimeMs, uint32_t offTimeMs, uint32_t totalDurationMs) {
    // If you want only this effect, kill others
    killAllTasks();

    if (blinkTaskHandle_ == nullptr) {
        // Use the new struct
        BlinkWithDurationTaskParams* params = new BlinkWithDurationTaskParams{
            this, onTimeMs, offTimeMs, totalDurationMs
        };
        xTaskCreate(blinkWithDurationTask,
                    "LedManagerBlinkWithDurationTask",
                    2048,
                    params,
                    1,
                    &blinkTaskHandle_);
    }
}

/* ------------------ Task Implementations ------------------ */

void LedManager::fadeInTask(void* parameter) {
    FadeTaskParams* params = static_cast<FadeTaskParams*>(parameter);
    LedManager* led = params->ledMgr;
    uint32_t durationMs = params->durationMs;
    delete params;

    int startDuty = led->duty_;
    int endDuty   = LEDC_MAX_DUTY;
    int range     = endDuty - startDuty;

    if (range > 0) {
        unsigned int steps = range;
        unsigned long stepDelay = (steps > 0) ? ((durationMs / steps) > 0 ? (durationMs / steps) : 1) : 1;
        for (unsigned int i = 1; i <= steps; i++) {
            led->duty_ = startDuty + i;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(stepDelay));
        }
    }

    // Ensure final
    led->duty_ = endDuty;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    led->fadeInTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

void LedManager::fadeOutTask(void* parameter) {
    FadeTaskParams* params = static_cast<FadeTaskParams*>(parameter);
    LedManager* led = params->ledMgr;
    uint32_t durationMs = params->durationMs;
    delete params;

    int startDuty = led->duty_;
    int endDuty   = 0;
    int range     = startDuty - endDuty;

    if (range > 0) {
        unsigned int steps = range;
        unsigned long stepDelay = (steps > 0) ? (durationMs / steps) : 0;
        for (unsigned int i = 1; i <= steps; i++) {
            led->duty_ = startDuty - i;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(stepDelay));
        }
    }

    // Ensure final
    led->duty_ = endDuty;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    led->fadeOutTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

void LedManager::fadeInOutTask(void* parameter) {
    FadeTaskParams* params = static_cast<FadeTaskParams*>(parameter);
    LedManager* led       = params->ledMgr;
    uint32_t durationMs   = params->durationMs;

    // We can delete the heap-allocated params struct now
    delete params;

    // Indefinite loop
    while (true) {
        // ------------------ Fade IN ------------------
        {
            int startDuty = led->duty_;
            int endDuty   = LEDC_MAX_DUTY;
            int range     = endDuty - startDuty;

            if (range > 0) {
                unsigned int steps      = range;
                // Step delay is total fade duration / number of steps
                unsigned long stepDelay = (steps > 0) ? ((durationMs / steps) > 0 ? (durationMs / steps) : 1) : 1;

                for (unsigned int i = 1; i <= steps; i++) {
                    led->duty_ = startDuty + i;
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(pdMS_TO_TICKS(stepDelay));
                }
            }

            // Ensure final duty is truly max
            led->duty_ = endDuty;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }

        // ------------------ Fade OUT ------------------
        {
            int startDuty = led->duty_;
            int endDuty   = 0;
            int range     = startDuty - endDuty;

            if (range > 0) {
                unsigned int steps      = range;
                unsigned long stepDelay = (steps > 0) ? ((durationMs / steps) > 0 ? (durationMs / steps) : 1) : 1;

                for (unsigned int i = 1; i <= steps; i++) {
                    led->duty_ = startDuty - i;
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(pdMS_TO_TICKS(stepDelay));
                }
            }

            // Ensure final duty is truly zero
            led->duty_ = endDuty;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }

        // Then it repeats indefinitely
    }

    // We never reach this point in an infinite loop, but for completeness:
    led->fadeInOutTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}


void LedManager::blinkTask(void* parameter) {
    BlinkTaskParams* params = static_cast<BlinkTaskParams*>(parameter);
    LedManager* led = params->ledMgr;
    uint32_t onTimeMs  = params->onTimeMs;
    uint32_t offTimeMs = params->offTimeMs;
    delete params;

    // Infinite blink
    while (true) {
        // ON
        led->duty_ = LEDC_MAX_DUTY;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(onTimeMs));

        // OFF
        led->duty_ = 0;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(offTimeMs));
    }

    led->blinkTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

/* ------------------ NEW TASK for blink-with-duration ------------------ */
void LedManager::blinkWithDurationTask(void* parameter) {
    BlinkWithDurationTaskParams* params = static_cast<BlinkWithDurationTaskParams*>(parameter);
    LedManager* led = params->ledMgr;
    uint32_t onTimeMs  = params->onTimeMs;
    uint32_t offTimeMs = params->offTimeMs;
    uint32_t totalDurationMs = params->totalDurationMs;
    delete params;

    uint32_t elapsed = 0;
    while (elapsed < totalDurationMs) {
        // Turn LED ON
        led->duty_ = LEDC_MAX_DUTY;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        // Wait for onTime
        if (elapsed + onTimeMs > totalDurationMs) {
            // If turning ON would exceed totalDuration, only wait partially
            vTaskDelay(pdMS_TO_TICKS(totalDurationMs - elapsed));
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(onTimeMs));
            elapsed += onTimeMs;
        }

        // Turn LED OFF
        led->duty_ = 0;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        // Wait for offTime
        if (elapsed + offTimeMs > totalDurationMs) {
            vTaskDelay(pdMS_TO_TICKS(totalDurationMs - elapsed));
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(offTimeMs));
            elapsed += offTimeMs;
        }
    }

    // Ensure LED is OFF at the end
    led->duty_ = 0;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led->duty_);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // Clear handle and self-delete
    led->blinkTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

/* ------------------ Utility & Configuration ------------------ */

void LedManager::killAllTasks() {
    if (fadeInTaskHandle_ != nullptr) {
        vTaskDelete(fadeInTaskHandle_);
        fadeInTaskHandle_ = nullptr;
    }
    if (fadeOutTaskHandle_ != nullptr) {
        vTaskDelete(fadeOutTaskHandle_);
        fadeOutTaskHandle_ = nullptr;
    }
    if (blinkTaskHandle_ != nullptr) {
        vTaskDelete(blinkTaskHandle_);
        blinkTaskHandle_ = nullptr;
    }
    if (fadeInOutTaskHandle_ != nullptr) {
        vTaskDelete(fadeInOutTaskHandle_);
        fadeInOutTaskHandle_ = nullptr;
    }
}

void LedManager::configLedc() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_cfg = {
        .gpio_num       = LED_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_cfg);
}
