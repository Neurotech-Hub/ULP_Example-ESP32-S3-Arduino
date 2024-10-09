// wakeup stub: https://github.com/espressif/arduino-esp32/issues/4266
// generic ULP: https://lang-ship.com/blog/work/esp32-ulp-l06/

#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/driver/gpio/include/driver/rtc_io.h"
#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/esp_hw_support/include/esp_sleep.h"
#include "esp32s3/ulp.h"
#include "/Users/gaidica/Documents/Arduino/hardware/espressif/esp32/tools/esp32-arduino-libs/esp32s3/include/soc/esp32s3/include/soc/rtc_io_reg.h"

#define GPIO_SENSOR_PIN GPIO_NUM_18  // GPIO pin connected to the sensor
#define RTC_GPIO_INDEX 18            // Serial.println(rtc_io_number_get(GPIO_SENSOR_PIN));

#define LED_PIN GPIO_NUM_13
#define LED_GPIO_INDEX 13

#define ULP_GPIO_STATE_ADDR 0
// #define ULP_COUNTER_ADDR 0    // Address in RTC_SLOW_MEM for the counter
#define ULP_THRESHOLD_ADDR 1  // Address in RTC_SLOW_MEM for the threshold

static size_t RTC_DATA_ATTR boots;
static size_t RTC_DATA_ATTR max_boots;

// Function which runs after exit from deep sleep
static void RTC_IRAM_ATTR esp_wake_stub_entry();

const int ulp_threshold_value = 4;  // Define the threshold as a constant integer

const ulp_insn_t ulp_program[] = {
    // Initialize transition counter and previous state
    I_MOVI(R3, 0),  // R3 <- 0 (reset the transition counter)
    I_MOVI(R2, 0),  // R2 <- 0 (previous state, assume LOW initially)

    // Main loop
    M_LABEL(1),

    // Read GPIO18 state (bit 28) via RTC_GPIO_IN_REG
    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S),

    I_WR_REG(RTC_GPIO_OUT_REG, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, R0),

    // Save the current state in a temporary register (R1)
    I_MOVR(R1, R0),  // R1 <- R0 (store current GPIO state temporarily)

    // Compare current state (R1) with previous state (R2)
    I_SUBR(R0, R1, R2),  // R0 = current state (R1) - previous state (R2)

    // If the state has not changed, skip the increment and continue holding
    I_BL(5, 1),  // If R0 == 0 (no state change), skip the next 6 instructions

    // Increment the transition counter (only if a transition is detected)
    I_ADDI(R3, R3, 1),  // Increment R3 by 1 (transition detected)

    // Store the current state in R2 (for comparison in the next iteration)
    I_MOVR(R2, R1),  // R2 <- R1 (store the current state for the next iteration)

    // Store the transition counter in RTC_SLOW_MEM[15]
    I_MOVI(R1, 15),   // Set R1 to address RTC_SLOW_MEM[15]
    I_ST(R3, R1, 0),  // Store R3 (counter) into RTC_SLOW_MEM[15]

    // Delay to slow down the ULP program (adjust delay value if needed)
    I_DELAY(0xFFFF),  // Introduce a delay to avoid fast looping

    // Loop back to the start
    M_BX(1),  // Loop back to label 1
};


// works for LED, delay is very short
// const ulp_insn_t ulp_program[] = {
//   // Main loop
//   M_LABEL(1),

//   // Read GPIO18 state (bit 28) via RTC_GPIO_IN_REG
//   I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S),

//   I_DELAY(65535),  // Introduce a delay to avoid fast looping

//   // If GPIO18 is HIGH, turn OFF the LED (connected to GPIO19)
//   M_BG(2, 0),  // If R0 > 0 (GPIO18 is HIGH), branch to label 2

//   // GPIO18 is LOW, turn ON the LED (connected to GPIO19, bit 19)
//   I_WR_REG(RTC_GPIO_OUT_REG, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, 1),  // Set GPIO19 to HIGH (LED ON)
//   M_BX(1),                                                                                                    // Loop back to label 1

//   // Label 2: GPIO18 is HIGH, turn OFF the LED
//   M_LABEL(2),
//   I_WR_REG(RTC_GPIO_OUT_REG, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, LED_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, 0),  // Set GPIO19 to LOW (LED OFF)
//   M_BX(1),                                                                                                    // Loop back to label 1
// };




// works! can not use RTC_SLOW_MEM[x] where ULP program is in x
// const ulp_insn_t ulp_program[] = {
//   // Initialize memory address for RTC_SLOW_MEM[1]
//   I_MOVI(R3, 0),  // R3 <- 0 (reset the transition counter)
//   I_MOVI(R2, 15),  // Set R2 to address RTC_SLOW_MEM[1]

//   // Main loop
//   M_LABEL(1),

//   // Read GPIO18 state (bit 28) via RTC_GPIO_IN_REG
//   I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S),

//   // If GPIO18 is HIGH, skip increment and continue
//   M_BG(1, 0),  // If R0 > 0 (GPIO18 is HIGH), go back to the start of the loop

//   // GPIO18 is LOW, check if counter is about to overflow
//   I_MOVI(R1, 0xFFFF),  // Load 0xFFFF into R1
//   I_SUBR(R0, R1, R3),  // R0 = 0xFFFF - R3 (check if the counter is about to overflow)
//   I_BL(2, 1),          // If R0 < 1 (i.e., R3 == 0xFFFF), skip the increment to avoid overflow

//   // Increment the transition counter
//   I_ADDI(R3, R3, 1),  // Increment R3 by 1

//   // Store the transition counter in RTC_SLOW_MEM[1]
//   I_ST(R3, R2, 0),  // Store R3 (counter) into RTC_SLOW_MEM[1]

//   // Delay to slow down the ULP program (adjust delay value if needed)
//   I_DELAY(0xFFFF),  // Introduce a delay to avoid fast looping

//   // Go back to check GPIO18 again
//   M_BX(1),  // Loop back to label 1 to keep checking

//   // Label 2: Skip increment to avoid overflow
//   M_LABEL(2),
//   I_HALT(),  // Halt the ULP program if overflow occurs (or handle it accordingly)
// };




// SIMPLE INCREMEMT TO READ R3
// works with: memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);  // Clear the RTC_SLOW_MEM
// const ulp_insn_t ulp_program[] = {
//   // Initialize transition counter (R3 <- 0)
//   I_MOVI(R3, 0),  // Set R3 to 0 (counter)
//   I_MOVI(R2, 1),  // Set R2 to address RTC_SLOW_MEM[1]

//   // Increment 1
//   I_ADDI(R3, R3, 1),  // Increment R3 by 1
//   I_ST(R3, R2, 0),    // Store R3 in RTC_SLOW_MEM[1]

//   // Increment 2
//   I_ADDI(R3, R3, 1),  // Increment R3 by 1
//   I_ST(R3, R2, 0),    // Store R3 in RTC_SLOW_MEM[1]

//   // Increment 3
//   I_ADDI(R3, R3, 1),  // Increment R3 by 1
//   I_ST(R3, R2, 0),    // Store R3 in RTC_SLOW_MEM[1]

//   // Halt the ULP program
//   I_HALT(),  // Halt the ULP program
// };




// WAKE UP ON LOW
// const ulp_insn_t ulp_program[] = {
//   // Label to loop back to
//   M_LABEL(1),

//   // Read GPIO18 state (bit 28) via RTC_GPIO_IN_REG
//   I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S, RTC_GPIO_INDEX + RTC_GPIO_OUT_DATA_S),

//   // Check if GPIO18 is low (R0 < 1). If low, wake up the CPU.
//   M_BL(2, 1),  // If R0 < 1 (i.e., if GPIO18 is low, branch to wakeup)

//   // Loop back to the start (label 1) if GPIO18 is not low
//   M_BX(1),  // Unconditional branch back to label 1

//   // Label 2: Wake up the main CPU
//   M_LABEL(2),

//   // Wake up the main CPU
//   I_WAKE(),  // Wake up the main CPU
//   I_HALT(),  // Halt the ULP program after waking the CPU
// };

static void RTC_IRAM_ATTR esp_wake_stub_entry() {
  // Increment the Boot counter
  boots++;
}

/**
 * @brief Set wake stub entry to default `esp_wake_stub_entry`
 */
// void esp_set_deep_sleep_wake_stub_default_entry(void);

void init_ulp_program() {
  size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(0, ulp_program, &size);
  // Reserve memory starting after the ULP program for data
  memset(&RTC_SLOW_MEM[size], 0, CONFIG_ULP_COPROC_RESERVE_MEM - (size * sizeof(uint32_t)));

  // Serial.print("ULP Size: ");
  // Serial.println(size);

  // Initialize GPIO for ULP to monitor
  rtc_gpio_init(GPIO_SENSOR_PIN);
  rtc_gpio_set_direction(GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_SENSOR_PIN);     // Enable the pull-up resistor
  rtc_gpio_pulldown_dis(GPIO_SENSOR_PIN);  // Disable the pull-down resistor
  // gpio_hold_en(GPIO_SENSOR_PIN);           // hold, likely do not need to disable
  rtc_gpio_hold_en(GPIO_SENSOR_PIN);

  rtc_gpio_init(LED_PIN);
  rtc_gpio_set_direction(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  esp_sleep_enable_timer_wakeup(5000 * 1000);  // debug

  // Check if the ESP32 woke up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_ULP) {
    // The device woke up from ULP, handle wakeup
    Serial.println("ULP woke up the processor!");

    // uint32_t gpio_state = RTC_SLOW_MEM[1] & 0xFFFF;  // Mask to only get 1 bit (GPIO state)
    // Serial.printf("GPIO18 state: %u\n", gpio_state);

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
    // RTC_SLOW_MEM[ULP_THRESHOLD_ADDR] = ulp_threshold_value;  // Set threshold value

    // for (int i = 0; i < 16; i++) {                // Adjust the loop limit to print more or fewer values
    //   uint32_t value = RTC_SLOW_MEM[i] & 0xFFFF;  // Read 16-bit value from RTC_SLOW_MEM
    //   Serial.printf("RTC_SLOW_MEM[%d]: %u\n", i, value);
    // }

    uint32_t value = RTC_SLOW_MEM[15] & 0xFFFF;
    Serial.printf("Count: %u\n", value - 1);

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
  /// We shouldn't be in the loop since we're going to deep sleep
  Serial.println("Unexpectedly woke up without ULP.");
  delay(1000);
}
