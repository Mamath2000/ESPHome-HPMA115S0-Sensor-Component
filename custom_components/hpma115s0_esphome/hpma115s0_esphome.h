#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace hpma115S0_esphome {

static const uint8_t SELECT_COMM_CMD = 0X88;

class HPMA115S0Component : public PollingComponent, public uart::UARTDevice {
 public:
  HPMA115S0Component() = default;

  void set_pm_2_5_sensor(sensor::Sensor *pm_2_5_sensor) { pm_2_5_sensor_ = pm_2_5_sensor; }
  void set_pm_10_0_sensor(sensor::Sensor *pm_10_0_sensor) { pm_10_0_sensor_ = pm_10_0_sensor; }

  void set_aqi_2_5_sensor(sensor::Sensor *aqi_2_5_sensor) { aqi_2_5_sensor_ = aqi_2_5_sensor; }
  void set_aqi_10_0_sensor(sensor::Sensor *aqi_10_0_sensor) { aqi_10_0_sensor_ = aqi_10_0_sensor; }

  void set_pm_4_0_sensor(sensor::Sensor *pm_4_0_sensor) { pm_4_0_sensor_ = pm_4_0_sensor; }
  void set_pm_1_0_sensor(sensor::Sensor *pm_1_0_sensor) { pm_1_0_sensor_ = pm_1_0_sensor; }
  void set_adjustment_coefficient_sensor(sensor::Sensor *adjustment_coefficient_sensor) {
    adjustment_coefficient_sensor_ = adjustment_coefficient_sensor;
  }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  bool start_measurement(void);
  bool stop_measurement(void);
  bool stop_autosend(void);
  bool enable_autosend(void);
  bool set_adjustment_coefficient(uint8_t adjustment_coefficient);
  bool read_adjustment_coefficient();

 protected:
  sensor::Sensor *pm_2_5_sensor_{nullptr};
  sensor::Sensor *pm_10_0_sensor_{nullptr};

  sensor::Sensor *aqi_2_5_sensor_{nullptr};
  sensor::Sensor *aqi_10_0_sensor_{nullptr};

  sensor::Sensor *pm_4_0_sensor_{nullptr};
  sensor::Sensor *pm_1_0_sensor_{nullptr};
  sensor::Sensor *adjustment_coefficient_sensor_{nullptr};

  float p25 = 0;
  float p10 = 0;
  float p4 = 0;
  float p1 = 0;

  bool launchSuccess = false;
  bool autosend_mode_ = false;
  bool measurement_running_ = true;
  long waitLast = 0;
  long waitTime = 300;
  uint32_t autosend_wait_time_ms_ = 1500;
  int setup_SAS = 0;
  int setup_SM = 0;
  int setupTries = 0;
  uint8_t adjustment_coefficient_ = 100;

  int comWait(bool start, int minDataToRead);
  void flush_input_();
  bool read_ack_(int *status_code);
  void publish_adjustment_coefficient_();
  bool wait_for_data_(size_t min_data, uint32_t timeout_ms);
  bool read_autosend_values_(float *p25, float *p10, float *p4, float *p1);
  bool read_values(float *p25, float *p10, float *p4, float *p1);
  uint16_t calculate_autosend_checksum_(uint8_t len_msb, uint8_t len_lsb, const uint8_t *frame, size_t frame_len_without_checksum) const;
  float calcAQI2_5() const;
  float calcAQI10() const;
};

}  // namespace hpma115S0_esphome
}  // namespace esphome