#include "esp32/ulp.h"
#include "esp_sleep.h" 
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "soc/sens_reg.h"

#define ULP_DATA_OFFSET                     36
#define CONFIG_ESP32_ULP_COPROC_RESERVE_MEM 1024
#define HIGH_ADC_THRESHOLD                  1700
#define LOW_ADC_THRESHOLD                   1600
#define PULSES_PER_LITRE                    2.0f
#define TAG                                 "PulseCounter"


_Static_assert(ULP_DATA_OFFSET < CONFIG_ESP32_ULP_COPROC_RESERVE_MEM/4 - 6,
        "ULP_DATA_OFFSET is set too high, or CONFIG_ESP32_ULP_COPROC_RESERVE_MEM is not sufficient");

static inline uint16_t ulp_data_read(size_t offset)
{
    return RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] & 0xffff;
}

static inline void ulp_data_write(size_t offset, uint16_t value)
{
    RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] = value;
}

class PulseCounter : public PollingComponent, public Sensor
{
private:
  void start_ulp_program()
  {
    // Start ULP
    ESP_LOGD(TAG, "Starting ULP program");
    ESP_ERROR_CHECK( ulp_run(0) );
    ESP_LOGD(TAG, "ULP Program started");
  }

  void load_ulp_program()
  {
    ESP_LOGD(TAG, "Loading ULP program");
    // Clear the part of RTC_SLOW_MEM reserved for the ULP. Makes debugging easier.
    memset(RTC_SLOW_MEM, 0, CONFIG_ESP32_ULP_COPROC_RESERVE_MEM);

    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();
    rtc_gpio_init(GPIO_NUM_36);

    // Initialize shared mem placeholders
    ulp_data_write(0, 0); // last_edge
    ulp_data_write(1, 0); // pulse_count
    ulp_data_write(2, 0); // run_counter
    ulp_data_write(3, 0); // last_reading

    // ULP Program that checks ADC (GPIO36) to count pulses (sine wave using uppper and lower thresholds)
    const ulp_insn_t program[] = {
        M_LABEL(1), // Label1: Start
          I_MOVI(R3, ULP_DATA_OFFSET),// Reference to RTC_SLOW_MEM region
          I_LD(R2, R3, 2),            // Load run counter into R2 in offset 2
          I_ADDI(R2, R2, 1),          // Increment run counter
          I_ST(R2, R3, 2),            // Store run counter in offset 2
          I_MOVI(R0, 0),              // Reset R0
          I_MOVI(R2, 0),              // Reset R2
        M_LABEL(2),                   // Label2: Measure
          I_ADDI(R0, R0, 1),          // Increment sample counter
          I_ADC(R1, 0, 0),            // Read from ADC Channel0, pad1 (GPIO36)
          I_ADDR(R2, R2, R1),         // Accumulate ADC readings in R2
        M_BL(2, 4),                   // Jump to Label2 if sample counter (R0) < 4
        I_RSHI(R0, R2, 2),            // Divide accumulated readings by sample counts (4) and store in R0
        I_MOVI(R3, ULP_DATA_OFFSET),  // Reference to RTC_SLOW_MEM region
        I_ST(R0, R3, 3),              // Store last reading in offset 3
        M_BL(3, LOW_ADC_THRESHOLD),   // Jump to Label3 if R0 >=  LOW_ADC_THRESHOLD
        M_BGE(4, HIGH_ADC_THRESHOLD), // Jump to Label4 if R0 >=  HIGH_ADC_THRESHOLD
        I_HALT(),                     // No edge detected, so exit
        M_LABEL(3),                   // Label3: LOW Edge Detected
          I_MOVI(R2, 0),              // Load current edge LOW(0) to R2
          I_MOVI(R1, ULP_DATA_OFFSET),// Load ULP offset to memory region
          I_LD(R3, R1, 0),            // Load previous_edge (1=HIGH, 0=LOW) from RTC_SLOW_MEM in offset:0
          I_SUBR(R0, R3, R2),         // Compare and result in R0
          M_BGE(5, 1),                // If result was >=1 then cycle complete
          I_HALT(),                   // Cycle not complete, so exit
        M_LABEL(4),                   // Label4: HIGH Edge Detected
          I_MOVI(R2, 1),              // Load current edge LOW(0) to R2
          I_MOVI(R1, ULP_DATA_OFFSET),// Load ULP offset to memory region
          I_LD(R3, R1, 0),            // Load previous_edge (1=HIGH, 0=LOW) from RTC_SLOW_MEM in offset:0
          I_SUBR(R0, R2, R3),         // Compare and result in R0
          M_BGE(5, 1),                // If result was >=1 then cycle complete
          I_HALT(),                   // Cycle not complete, so exit
        M_LABEL(5),                   // Label5: Sine wave cycle complete
          // Store current edge as previous_edge in memory
          I_MOVI(R1, ULP_DATA_OFFSET),// Load ULP offset to memory region
          I_ST(R2, R1, 0),            // previous_edge (1=HIGH, 0=LOW) to RTC_SLOW_MEM in offset:0
          // Increment pulse count in memory
          I_LD(R3, R1, 1),            // pulse_count from RTC_SLOW_MEM in offset:1
          I_ADDI(R3, R3, 1),          // Increment pulse count and result in R0
          I_ST(R3, R1, 1),            // pulse_count to RTC_SLOW_MEM in offset:1
        I_HALT()                      // End
    };

     // Load ULP program into RTC_SLOW_MEM, at offset 0
    size_t size = sizeof(program)/sizeof(ulp_insn_t);
    ESP_ERROR_CHECK( ulp_process_macros_and_load(0, program, &size) );
    assert(size < ULP_DATA_OFFSET && "ULP_DATA_OFFSET needs to be greater or equal to the program size");

    // Set ULP wakeup period
    const uint32_t sleep_cycles = 80000 / 2; // clock_hz/measurements_per_sec = 8Mhz/3
    REG_WRITE(SENS_ULP_CP_SLEEP_CYC0_REG, sleep_cycles);
    ESP_LOGD(TAG, "Loaded ULP program to memory");
  }

public:
  // Poll every 5s to update state
  PulseCounter() : PollingComponent(5000) {}

  void setup() override
  {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ESP_LOGD(TAG,"ULP wakeup\n");
    } else if(cause == ESP_SLEEP_WAKEUP_TIMER ) {
        ESP_LOGD(TAG,"Timer wake up\n");
    } else  {
        ESP_LOGD(TAG,"Not ULP wakeup\n");
        load_ulp_program();
    }

    // This will be called by App.setup()
    start_ulp_program();
  }

  void update() override
  {
    // Read last edge from RTC Slow mem
    uint16_t last_edge = ulp_data_read(0);

    // Read pulse count from RTC Slow mem
    uint16_t pulse_count = ulp_data_read(1);
    float litres = pulse_count / PULSES_PER_LITRE;
    auto mqtt_sensor = MQTTSensorComponent(this);
    if(mqtt_sensor.publish_state(litres))
    {
      ulp_data_write(1, 0);
      ESP_LOGD(TAG, "Successfully sent data via MQTT");  
    }else
    {
      ESP_LOGD(TAG, "Unable to send data via MQTT");  
    }

    // Read run counter from RTC Slow mem
    uint16_t run_counter = ulp_data_read(2);
    ulp_data_write(2, 0);

    // Read last_reading from RTC Slow mem
    uint16_t last_reading = ulp_data_read(3);
    ulp_data_write(3, 0);

    ESP_LOGD(TAG, "Litres: %f, Pulse Count: %d, Last Edge: %d, Last Avg. Reading: %d, ULP Run Count: %d \n", litres, pulse_count, last_edge, last_reading, run_counter);
  }
};