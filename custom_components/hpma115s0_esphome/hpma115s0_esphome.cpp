#include "esphome/core/log.h"
#include "hpma115s0_esphome.h"

namespace esphome {
namespace hpma115S0_esphome {

#define byte uint8_t
static const char *const TAG = "hpma115s0.sensor";

void HPMA115S0Component::setup() {
  if (read_autosend_values_(&p25, &p10, &p4, &p1)) {
    autosend_mode_ = true;
    launchSuccess = true;
    measurement_running_ = true;
    ESP_LOGI(TAG, "Detected autosend frame 0x42 0x4D; using passive UART reads.");
    publish_adjustment_coefficient_();
    return;
  }

  while (setupTries < 3) {
    setupTries++;
    flush_input_();
    if (stop_autosend()) {
      if (start_measurement()) {
        launchSuccess = true;
        break;
      }
    }
  }

  publish_adjustment_coefficient_();
}

void HPMA115S0Component::dump_config() {
  if (launchSuccess) {
    if (autosend_mode_) {
      ESP_LOGCONFIG(TAG, "Sensor found in autosend mode (0x42 0x4D). Will read and log values.");
    } else {
      ESP_LOGCONFIG(TAG, "Sensor found after %i tries. Will read and log values.", setupTries);
    }
  } else {
    ESP_LOGW(TAG,
             "Initial HPMA probe did not answer during setup (stop_autosend=%i, start_measurement=%i). Will retry during the next read cycle.",
             setup_SAS, setup_SM);
  }
}

float HPMA115S0Component::get_setup_priority() const { return setup_priority::LATE; }

void HPMA115S0Component::update() {
  if (!measurement_running_) {
    return;
  }

  if (read_values(&p25, &p10, &p4, &p1)) {
    if (this->pm_2_5_sensor_ != nullptr) {
      this->pm_2_5_sensor_->publish_state(p25);
    }
    if (this->pm_10_0_sensor_ != nullptr) {
      this->pm_10_0_sensor_->publish_state(p10);
    }
    if (aqi_2_5_sensor_ != nullptr) {
      (*aqi_2_5_sensor_).publish_state(calcAQI2_5());
    }
    if (aqi_10_0_sensor_ != nullptr) {
      (*aqi_10_0_sensor_).publish_state(calcAQI10());
    }
    if (pm_4_0_sensor_ != nullptr) {
      (*pm_4_0_sensor_).publish_state(p4);
    }
    if (pm_1_0_sensor_ != nullptr) {
      (*pm_1_0_sensor_).publish_state(p1);
    }
    launchSuccess = true;
  } else if (launchSuccess) {
    ESP_LOGE(TAG, "Read Values Failed - See Previous Message");
  } else {
    ESP_LOGI(TAG, "Not Updating. HPMA115S0 sensor was not found.");
  }
}

int HPMA115S0Component::comWait(bool start, int minDataToRead) {
  if (start) {
    waitLast = millis();
    return 0;
  } else if (this->available() >= minDataToRead) {
    return 1;
  } else if ((millis() - waitLast) >= waitTime) {
    return 2;
  }
  return 3;
}

void HPMA115S0Component::flush_input_() {
  while (this->available() > 0) {
    this->read();
  }
}

bool HPMA115S0Component::read_ack_(int *status_code) {
  const uint32_t start = millis();
  int state = 0;
  uint8_t candidate = 0;

  while ((millis() - start) < waitTime) {
    while (this->available() > 0) {
      const uint8_t current = this->read();
      if (state == 0) {
        if (current == 0xA5 || current == 0x96) {
          candidate = current;
          state = 1;
        }
        continue;
      }

      if (current == candidate) {
        if (candidate == 0xA5) {
          if (status_code != nullptr) {
            *status_code = 1;
          }
          return true;
        }

        if (status_code != nullptr) {
          *status_code = 3;
        }
        return false;
      }

      if (current == 0xA5 || current == 0x96) {
        candidate = current;
        state = 1;
      } else {
        state = 0;
      }
    }
  }

  if (status_code != nullptr) {
    *status_code = 2;
  }
  return false;
}

void HPMA115S0Component::publish_adjustment_coefficient_() {
  if (adjustment_coefficient_sensor_ != nullptr) {
    adjustment_coefficient_sensor_->publish_state(adjustment_coefficient_);
  }
}

bool HPMA115S0Component::wait_for_data_(size_t min_data, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    if (this->available() >= min_data) {
      return true;
    }
  }
  return false;
}

uint16_t HPMA115S0Component::calculate_autosend_checksum_(uint8_t len_msb, uint8_t len_lsb, const uint8_t *frame, size_t frame_len_without_checksum) const {
  uint16_t sum = 0x42 + 0x4D + len_msb + len_lsb;
  for (size_t index = 0; index < frame_len_without_checksum; index++) {
    sum += frame[index];
  }
  return sum;
}

byte calculateChecksum(byte HEAD, byte LEN, byte *messageBuffer) {
  int sum = 0;
  for (int index = 0; index < LEN; index++) {
    sum += messageBuffer[index];
  }
  return (0x10000 - HEAD - LEN - sum) % 0x100;
}

bool HPMA115S0Component::read_autosend_values_(float *p25, float *p10, float *p4, float *p1) {
  const uint32_t start = millis();
  int state = 0;

  while ((millis() - start) < autosend_wait_time_ms_) {
    while (this->available()) {
      const byte current = this->read();
      if (state == 0) {
        if (current == 0x42) {
          state = 1;
        }
        continue;
      }

      if (current == 0x4D) {
        if (!wait_for_data_(2, autosend_wait_time_ms_)) {
          ESP_LOGD(TAG, "Autosend header found but length bytes timed out");
          return false;
        }

        const byte len_msb = this->read();
        const byte len_lsb = this->read();
        const uint16_t frame_len = (len_msb << 8) | len_lsb;
        if (frame_len < 8 || frame_len > 30) {
          ESP_LOGD(TAG, "Unexpected autosend frame length: %u", frame_len);
          return false;
        }

        byte frame[32];
        if (!wait_for_data_(frame_len, autosend_wait_time_ms_)) {
          ESP_LOGD(TAG, "Autosend frame payload timed out");
          return false;
        }
        if (!this->read_array(frame, frame_len)) {
          ESP_LOGE(TAG, "Autosend frame read failed");
          return false;
        }

        const uint16_t received_checksum = (frame[frame_len - 2] << 8) | frame[frame_len - 1];
        const uint16_t expected_checksum = calculate_autosend_checksum_(len_msb, len_lsb, frame, frame_len - 2);
        if (received_checksum != expected_checksum) {
          ESP_LOGE(TAG, "Autosend checksum mismatch - expected 0x%04X got 0x%04X", expected_checksum, received_checksum);
          return false;
        }

        const uint16_t data0 = (frame[0] << 8) | frame[1];
        const uint16_t data1 = (frame[2] << 8) | frame[3];
        const uint16_t data2 = (frame[4] << 8) | frame[5];
        const uint16_t data3 = (frame[6] << 8) | frame[7];

        *p1 = NAN;
        *p4 = NAN;
        *p25 = data1;
        *p10 = data2;

        ESP_LOGD(TAG, "Autosend raw words d0=%u d1=%u d2=%u d3=%u => PM2.5=%.0f PM10=%.0f", data0, data1, data2, data3, *p25, *p10);
        return true;
      }

      state = current == 0x42 ? 1 : 0;
    }
  }

  return false;
}

bool HPMA115S0Component::read_values(float *p25, float *p10, float *p4, float *p1) {
  if (autosend_mode_) {
    return read_autosend_values_(p25, p10, p4, p1);
  }

  if (this->available() > 0 && read_autosend_values_(p25, p10, p4, p1)) {
    autosend_mode_ = true;
    measurement_running_ = true;
    ESP_LOGI(TAG, "Switching to autosend frame parser based on observed UART data.");
    return true;
  }

  flush_input_();

  byte read_particle[] = {0x68, 0x01, 0x04, 0x93};
  this->write_array(read_particle, sizeof(read_particle));

  for (comWait(true, 2); comWait(false, 2) == 3;) {
  }
  if (!(this->available() >= 2)) {
    ESP_LOGE(TAG, "Read Values Failed - Serial Timeout to sensor");
    ESP_LOGD(TAG, "Available: %i", this->available());
    return false;
  }

  byte HEAD = this->read();
  byte LEN = this->read();
  if (HEAD != 0x40 || (LEN != 0x05 && LEN != 0x0D)) {
    ESP_LOGE(TAG, "Invalid Header - Check debug data if this happens again");
    ESP_LOGE(TAG, "HEAD %i LEN %i", HEAD, LEN);
    return false;
  }

  for (comWait(true, LEN + 1); comWait(false, LEN + 1) == 3;) {
  }
  if (!(this->available() >= LEN + 1)) {
    ESP_LOGE(TAG, "Most likely NACK as only 2 bytes recieved - Check debug data if this happens again");
    ESP_LOGE(TAG, "HEAD %i LEN %i", HEAD, LEN);
    return false;
  }

  byte messageBuffer[LEN];
  if (!this->read_array(messageBuffer, LEN)) {
    ESP_LOGE(TAG, "Read Values Failed - Serial Buffer Error");
    return false;
  }

  byte CS = this->read();
  if (CS != calculateChecksum(HEAD, LEN, messageBuffer)) {
    ESP_LOGE(TAG, "Checksum Mismatch - Check debug data if this happens again");
    return false;
  }

  if (LEN == 0x05) {
    *p25 = messageBuffer[1] * 256 + messageBuffer[2];
    *p10 = messageBuffer[3] * 256 + messageBuffer[4];
  } else {
    *p25 = messageBuffer[3] * 256 + messageBuffer[4];
    *p10 = messageBuffer[7] * 256 + messageBuffer[8];
    *p4 = messageBuffer[5] * 256 + messageBuffer[6];
    *p1 = messageBuffer[1] * 256 + messageBuffer[2];
  }
  return true;
}

bool HPMA115S0Component::start_measurement(void) {
  ESP_LOGI(TAG, "Attempting to Start Measurement for HPMA115S0");
  flush_input_();
  uint8_t start_measurement[] = {0x68, 0x01, 0x01, 0x96};
  this->write_array(start_measurement, sizeof(start_measurement));

  if (!read_ack_(&setup_SM)) {
    return false;
  }

  measurement_running_ = true;
  launchSuccess = true;
  return true;
}

bool HPMA115S0Component::stop_measurement(void) {
  ESP_LOGI(TAG, "Attempting to Stop Measurement for HPMA115S0");
  flush_input_();
  byte stop_measurement[] = {0x68, 0x01, 0x02, 0x95};
  this->write_array(stop_measurement, sizeof(stop_measurement));
  int ack_status = 0;
  if (!read_ack_(&ack_status)) {
    return false;
  }

  measurement_running_ = false;
  return true;
}

bool HPMA115S0Component::stop_autosend(void) {
  ESP_LOGI(TAG, "Attempting to Stop Autosend for HPMA115S0");

  flush_input_();
  byte stop_autosend[] = {0x68, 0x01, 0x20, 0x77};
  this->write_array(stop_autosend, sizeof(stop_autosend));

  if (!read_ack_(&setup_SAS)) {
    return false;
  }

  autosend_mode_ = false;
  launchSuccess = true;
  return true;
}

bool HPMA115S0Component::enable_autosend(void) {
  ESP_LOGI(TAG, "Attempting to Enable Autosend for HPMA115S0");
  flush_input_();
  byte start_autosend[] = {0x68, 0x01, 0x40, 0x57};
  this->write_array(start_autosend, sizeof(start_autosend));
  int ack_status = 0;
  if (!read_ack_(&ack_status)) {
    return false;
  }

  autosend_mode_ = true;
  measurement_running_ = true;
  launchSuccess = true;
  return true;
}

bool HPMA115S0Component::set_adjustment_coefficient(uint8_t adjustment_coefficient) {
  if (adjustment_coefficient < 30 || adjustment_coefficient > 200) {
    ESP_LOGE(TAG, "Adjustment coefficient out of range: %u", adjustment_coefficient);
    return false;
  }

  byte payload[] = {0x08, adjustment_coefficient};
  const byte checksum = calculateChecksum(0x68, 0x02, payload);
  const byte command[] = {0x68, 0x02, 0x08, adjustment_coefficient, checksum};

  ESP_LOGI(TAG, "Setting customer adjustment coefficient to %u", adjustment_coefficient);
  flush_input_();
  this->write_array(command, sizeof(command));

  int ack_status = 0;
  if (!read_ack_(&ack_status)) {
    return false;
  }

  adjustment_coefficient_ = adjustment_coefficient;
  publish_adjustment_coefficient_();
  return true;
}

bool HPMA115S0Component::read_adjustment_coefficient() {
  const byte command[] = {0x68, 0x01, 0x10, 0x87};

  ESP_LOGI(TAG, "Reading customer adjustment coefficient");
  flush_input_();
  this->write_array(command, sizeof(command));

  const uint32_t start = millis();
  int state = 0;

  while ((millis() - start) < autosend_wait_time_ms_) {
    while (this->available() > 0) {
      const byte current = this->read();
      if (state == 0) {
        if (current == 0x40) {
          state = 1;
        }
        continue;
      }

      if (current != 0x02) {
        state = current == 0x40 ? 1 : 0;
        continue;
      }

      if (!wait_for_data_(3, autosend_wait_time_ms_)) {
        ESP_LOGE(TAG, "Adjustment coefficient response timed out");
        return false;
      }

      byte message_buffer[2];
      if (!this->read_array(message_buffer, 2)) {
        ESP_LOGE(TAG, "Adjustment coefficient response read failed");
        return false;
      }

      const byte checksum = this->read();
      if (message_buffer[0] != 0x10) {
        ESP_LOGE(TAG, "Unexpected adjustment response command: 0x%02X", message_buffer[0]);
        return false;
      }

      if (checksum != calculateChecksum(0x40, 0x02, message_buffer)) {
        ESP_LOGE(TAG, "Adjustment coefficient checksum mismatch");
        return false;
      }

      adjustment_coefficient_ = message_buffer[1];
      publish_adjustment_coefficient_();
      ESP_LOGI(TAG, "Customer adjustment coefficient is %u", adjustment_coefficient_);
      return true;
    }
  }

  ESP_LOGE(TAG, "Adjustment coefficient response timed out");
  return false;
}

float HPMA115S0Component::calcAQI2_5() const {
  if (p25 < 0.0) {
    return -1.0;
  }
  if (p25 < 12.0) {
    return (p25) / 12.0 * 50.0;
  }
  if (p25 < 35.4) {
    return (p25 - 12.0) * 50.0 / (35.4 - 12.0) + 50.0;
  }
  if (p25 < 55.4) {
    return (p25 - 35.4) * 50.0 / (55.4 - 35.4) + 100.0;
  }
  if (p25 < 150.4) {
    return (p25 - 55.4) * 50.0 / (150.4 - 55.4) + 150.0;
  }
  if (p25 < 250.4) {
    return (p25 - 150.4) + 200.0;
  }
  if (p25 < 350.4) {
    return (p25 - 250.4) + 300.0;
  }
  return (p25 - 350.4) + 400.0;
}

float HPMA115S0Component::calcAQI10() const {
  if (p10 < 0.0) {
    return -1.0;
  }
  if (p10 < 54.0) {
    return (p10) / 54.0 * 50.0;
  }
  if (p10 < 154.0) {
    return (p10 - 54.0) * 50.0 / (154.0 - 54.0) + 50.0;
  }
  if (p10 < 254.0) {
    return (p10 - 154.0) * 50.0 / (254.0 - 154.0) + 100.0;
  }
  if (p10 < 354.0) {
    return (p10 - 254.0) * 50.0 / (354.0 - 254.0) + 150.0;
  }
  if (p10 < 424.0) {
    return (p10 - 354.0) * 100.0 / (424.0 - 354.0) + 200.0;
  }
  if (p10 < 504.0) {
    return (p10 - 424.0) * 100.0 / (504.0 - 424.0) + 300.0;
  }
  return (p10 - 504.0) + 400.0;
}

}  // namespace hpma115S0_esphome
}  // namespace esphome