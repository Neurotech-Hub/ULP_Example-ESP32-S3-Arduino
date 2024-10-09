// wakeup stub: https://github.com/espressif/arduino-esp32/issues/4266

#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/driver/gpio/include/driver/rtc_io.h"
#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/esp_hw_support/include/esp_sleep.h"
#include "esp32s3/ulp.h"
#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/soc/esp32s3/include/soc/rtc_io_reg.h"

#define GPIO_SENSOR_PIN GPIO_NUM_18  // GPIO pin connected to the sensor
#define RTC_GPIO_INDEX 18            // Serial.println(rtc_io_number_get(GPIO_SENSOR_PIN));

#define ULP_GPIO_STATE_ADDR 0
// #define ULP_COUNTER_ADDR 0    // Address in RTC_SLOW_MEM for the counter
#define ULP_THRESHOLD_ADDR 1  // Address in RTC_SLOW_MEM for the threshold

static size_t RTC_DATA_ATTR boots;
static size_t RTC_DATA_ATTR max_boots;

// Function which runs after exit from deep sleep
static void RTC_IRAM_ATTR esp_wake_stub_entry();

const int ulp_threshold_value = 4;  // Define the threshold as a constant integer

const ulp_insn_t ulp_program[] = {
  // I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S),  // Read RTC GPIO state
  // Label to loop back to
  M_LABEL(1),

  // Read GPIO18 state (bit 28) via RTC_GPIO_IN_REG
  I_RD_REG(RTC_GPIO_IN_REG, 28, 28),  // Read GPIO18's state into R0 using bit 28

  // Check if GPIO18 is low (R0 < 1). If low, wake up the CPU.
  M_BL(2, 1),  // If R0 < 1 (i.e., if GPIO18 is low, branch to wakeup)

  // Loop back to the start (label 1) if GPIO18 is not low
  M_BX(1),  // Unconditional branch back to label 1

  // Label 2: Wake up the main CPU
  M_LABEL(2),

  // Wake up the main CPU
  I_WAKE(),  // Wake up the main CPU
  I_HALT(),  // Halt the ULP program after waking the CPU
};

static void RTC_IRAM_ATTR esp_wake_stub_entry() {
  // Increment the Boot counter
  boots++;
}

// const ulp_insn_t ulp_program[] = {
//   I_MOVI(R3, ULP_COUNTER_ADDR),  // Set the address for the counter in RTC_SLOW_MEM
//   I_LD(R0, R3, 0),               // Load the current counter value from RTC_SLOW_MEM

//   // Read GPIO18 state via RTC_CNTL registers
//   I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX, RTC_GPIO_INDEX),  // Read RTC GPIO state
//   I_BGE(1, 0),                                                // If GPIO18 is high, skip the next instruction

//   // GPIO18 went from high to low, increment counter
//   I_ADDI(R0, R0, 1),  // Increment the counter if GPIO18 is low
//   I_ST(R0, R3, 0),    // Store the incremented counter back into RTC_SLOW_MEM

//   // Load threshold value dynamically
//   I_MOVI(R3, ULP_THRESHOLD_ADDR),  // Set the address for the threshold in RTC_SLOW_MEM
//   I_LD(R2, R3, 0),                 // Load the threshold value into R2

//   I_SUBR(R0, R0, R2),  // Subtract threshold from the counter (R0)
//   I_BGE(1, 0),         // If counter >= threshold, branch to wakeup
//   I_HALT(),            // Halt if the counter is not yet threshold

//   I_WAKE(),  // Wake up the main CPU when counter reaches threshold
//   I_HALT()   // Halt the ULP program after waking the CPU
// };


/**
 * @brief Set wake stub entry to default `esp_wake_stub_entry`
 */
// void esp_set_deep_sleep_wake_stub_default_entry(void);

void init_ulp_program() {
  size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(0, ulp_program, &size);

  // Initialize GPIO for ULP to monitor
  rtc_gpio_init(GPIO_SENSOR_PIN);
  rtc_gpio_set_direction(GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_SENSOR_PIN);     // Enable the pull-up resistor
  rtc_gpio_pulldown_dis(GPIO_SENSOR_PIN);  // Disable the pull-down resistor
  // gpio_hold_en(GPIO_SENSOR_PIN);           // hold, likely do not need to disable
  rtc_gpio_hold_en(GPIO_SENSOR_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Check if the ESP32 woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_ULP) {
    // The device woke up from ULP, handle wakeup
    Serial.println("ULP woke up the processor!");

    uint32_t gpio_state = RTC_SLOW_MEM[1] & 0xFFFF;  // Mask to only get 1 bit (GPIO state)
    Serial.printf("GPIO18 state: %u\n", gpio_state);

    // Reset the counter in RTC_SLOW_MEM after wakeup
    // RTC_SLOW_MEM[ULP_COUNTER_ADDR] = 0;

    // Initialize ULP program but do NOT run it yet
    init_ulp_program();

    // Enable ULP wakeup source
    esp_sleep_enable_ulp_wakeup();

    // Now, start ULP program
    ulp_run(0);  // Start the ULP program

    // Go to deep sleep
    esp_deep_sleep_start();
  } else {
    // This is the first boot, perform full initialization
    Serial.println("Hello, ULP");
    boots = 0;

    // Set initial threshold in RTC memory
    RTC_SLOW_MEM[ULP_THRESHOLD_ADDR] = ulp_threshold_value;  // Set threshold value

    // Initialize ULP program but do NOT run it yet
    init_ulp_program();

    // Enable ULP wakeup source
    esp_sleep_enable_ulp_wakeup();

    // Now, start ULP program
    ulp_run(0);  // Start the ULP program

    // Go to deep sleep
    esp_deep_sleep_start();
  }
}

void loop() {
  // Should never reach here; deep sleep will always reset the ESP32
}
