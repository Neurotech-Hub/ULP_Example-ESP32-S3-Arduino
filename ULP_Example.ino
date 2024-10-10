// Matt Gaidica, PhD
// Neurotech Hub, Washington University in St. Louis
// https://neurotechhub.wustl.edu/

#include "esp32s3/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_io_reg.h"

#define GPIO_SENSOR_PIN GPIO_NUM_18  // GPIO pin connected to the sensor
#define RTC_GPIO_INDEX 18            // attain dynamically with: rtc_io_number_get(GPIO_SENSOR_PIN)

#define LED_PIN LED_BUILTIN

enum {
  EDGE_COUNT,
  SLOW_PROG_ADDR  // Program start address
};

// counts all state changes (LOW->HIGH, HIGH->LOW)
// divide by 2 for single transition type
const ulp_insn_t ulp_program[] = {
  // Initialize transition counter and previous state
  I_MOVI(R3, 0),  // R3 <- 0 (reset the transition counter)
  I_MOVI(R2, 1),  // R2 <- 0 (previous state, assume LOW initially)

  // Main loop
  M_LABEL(1),
  
  // Read RTC_GPIO_INDEX with RTC offset
  I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S),

  // Save the current state in a temporary register (R1)
  I_MOVR(R1, R0),  // R1 <- R0 (store current GPIO state temporarily)

  // Compare current state (R1) with previous state (R2)
  I_SUBR(R0, R1, R2),  // R0 = current state (R1) - previous state (R2)
  I_BL(5, 1),  // If R0 == 0 (no state change), skip instructions
  I_ADDI(R3, R3, 1),  // Increment R3 by 1 (transition detected)
  I_MOVR(R2, R1),  // R2 <- R1 (store the current state for the next iteration)

  // Store the transition counter
  I_MOVI(R1, EDGE_COUNT),  // Set R1 to address RTC_SLOW_MEM[1]
  I_ST(R3, R1, 0),         // Store it in RTC_SLOW_MEM

  // RTC clock on the ESP32-S3 is 17.5MHz, delay 0xFFFF = 3.74 ms
  I_DELAY(0xFFFF),  // debounce
  I_DELAY(0xFFFF),  // debounce
  I_DELAY(0xFFFF),  // debounce
  I_DELAY(0xFFFF),  // debounce
  I_DELAY(0xFFFF),  // debounce
  I_DELAY(0xFFFF),  // debounce

  M_BX(1),  // Loop back to label 1
};

void init_ulp_program() {
  memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);  // set to zeros, optional
  size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
  esp_err_t err = ulp_process_macros_and_load(SLOW_PROG_ADDR, ulp_program, &size);  // offset by PROG_ADDR

  if (err == ESP_OK) {
    // Serial.println("ULP program loaded successfully."); // optional
  } else if (err == ESP_ERR_NO_MEM) {
    Serial.println("Error: Not enough memory to load ULP program.");
  } else {
    Serial.printf("Error: ULP program load returned unexpected error %d\n", err);
  }

  // init GPIO for ULP to monitor
  rtc_gpio_init(GPIO_SENSOR_PIN);
  rtc_gpio_set_direction(GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(GPIO_SENSOR_PIN);     // enable the pull-up resistor
  rtc_gpio_pulldown_dis(GPIO_SENSOR_PIN);  // disable the pull-down resistor
  rtc_gpio_hold_en(GPIO_SENSOR_PIN);       // required to maintain pull-up
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // turn on during init

  Serial.begin(115200);
  delay(1000);  // Connect delay

  // show count, will init to a random value
  Serial.printf("Count: %u\n", RTC_SLOW_MEM[EDGE_COUNT] & 0xFFFF);

  init_ulp_program();  // Load the ULP program

  esp_sleep_enable_timer_wakeup(5000 * 1000);  // Wake up every 5 seconds

  ulp_run(SLOW_PROG_ADDR);  // Start the ULP program with offset

  digitalWrite(LED_PIN, LOW);  // turn off during sleep

  esp_deep_sleep_start();  // Enter deep sleep
}

void loop() {
}