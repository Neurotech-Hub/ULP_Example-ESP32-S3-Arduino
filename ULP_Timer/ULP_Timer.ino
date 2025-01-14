#include "esp32s3/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_periph.h"
#include "esp_sleep.h"

// Define timer period in microseconds
#define ULP_TIMER_PERIOD 100000 // 100ms
#define DEEP_SLEEP_TIME 5000000 // 5 seconds

// Memory locations in RTC memory
enum
{
    COUNTER_ADDR, // RTC memory location counter
    PROG_START    // Program start address
};

// ULP program source (written in assembly)
const ulp_insn_t ulp_program[] = {
    // Load counter value from RTC memory
    I_LD(R1, R0, COUNTER_ADDR), // Load value at COUNTER_ADDR into R1
    I_ADDI(R1, R1, 1),          // Increment R1
    I_ST(R1, R0, COUNTER_ADDR), // Store R1 back to COUNTER_ADDR
    I_HALT()                    // Halt until next timer wake
};

void setup()
{
    // Stop timer (in case it was running)
    ulp_timer_stop();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.begin(115200);
    delay(1000); // Allow time for serial connection

    // Check wake-up cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
    {
        // Only print counter value if we woke up from deep sleep
        Serial.printf("\nDeep sleep time: %d seconds\n", DEEP_SLEEP_TIME / 1000000);
        Serial.printf("ULP timer period: %d seconds\n", ULP_TIMER_PERIOD / 1000000);
        Serial.printf("Counter value: %d\n", (int)RTC_SLOW_MEM[COUNTER_ADDR]);
    }

    Serial.println("Initializing ULP timer program");

    // Initialize RTC memory
    RTC_SLOW_MEM[COUNTER_ADDR] = 0;

    // Load ULP program
    size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
    esp_err_t err = ulp_process_macros_and_load(PROG_START, ulp_program, &size);
    if (err != ESP_OK)
    {
        Serial.printf("Error loading ULP program: %d\n", err);
        return;
    }

    // Set ULP wake-up period
    err = ulp_set_wakeup_period(0, ULP_TIMER_PERIOD);
    if (err != ESP_OK)
    {
        Serial.printf("Error setting timer period: %d\n", err);
        return;
    }

    // Start ULP program
    err = ulp_run(PROG_START);
    if (err != ESP_OK)
    {
        Serial.printf("Error starting ULP program: %d\n", err);
        return;
    }

    // Configure deep sleep wake-up timer
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME); // 5 seconds in microseconds
    Serial.println("Entering deep sleep");
    Serial.flush();
    digitalWrite(LED_BUILTIN, LOW);

    // Enter deep sleep
    esp_deep_sleep_start();
}

void loop()
{
    // This will never run as we enter deep sleep in setup()
}
