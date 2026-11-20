#pragma once

#include <Arduino.h>

class RS485Modbus {
public:
  RS485Modbus(HardwareSerial& port, int8_t rx_pin, int8_t tx_pin, int8_t dir_pin = -1)
    : _serial(port), _rx_pin(rx_pin), _tx_pin(tx_pin), _dir_pin(dir_pin) {}

  void begin(uint32_t baud = 9600) {
    if (_dir_pin >= 0) {
      pinMode(_dir_pin, OUTPUT);
      digitalWrite(_dir_pin, LOW);
    }
    _serial.begin(baud, SERIAL_8N1, _rx_pin, _tx_pin);
  }

  static uint16_t calcCRC(const uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
      crc ^= (uint16_t)buf[pos];
      for (int i = 8; i != 0; i--) {
        if (crc & 0x0001) {
          crc = (crc >> 1) ^ 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  // Gửi frame thô và nhận phản hồi
  int sendRaw(const uint8_t* tx_data, int tx_len, uint8_t* rx_data, int rx_max, uint32_t timeout_ms = 300) {
    while (_serial.available()) _serial.read();

    if (_dir_pin >= 0) {
      digitalWrite(_dir_pin, HIGH);
      delayMicroseconds(50);
    }

    _serial.write(tx_data, tx_len);
    _serial.flush();

    if (_dir_pin >= 0) {
      digitalWrite(_dir_pin, LOW);
    }

    uint32_t start = millis();
    int rx_cnt = 0;
    while (millis() - start < timeout_ms && rx_cnt < rx_max) {
      if (_serial.available()) {
        rx_data[rx_cnt++] = _serial.read();
        start = millis(); // Reset timeout sau mỗi byte nhận được (inter-byte timeout)
      }
    }
    return rx_cnt;
  }

  // Đọc Holding Registers (0x03) hoặc Input Registers (0x04)
  // Kết quả trả về số lượng thanh ghi đọc được, lưu vào mảng dest
  int readRegisters(uint8_t slave, uint8_t func, uint16_t reg_addr, uint16_t count, uint16_t* dest, char* err_msg, size_t err_size, uint32_t timeout_ms = 350) {
    if (func != 3 && func != 4) {
      snprintf(err_msg, err_size, "Err: func must be 3 or 4");
      return -1;
    }
    if (count < 1 || count > 32) {
      snprintf(err_msg, err_size, "Err: count 1-32");
      return -1;
    }

    uint8_t frame[8];
    frame[0] = slave;
    frame[1] = func;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)(reg_addr & 0xFF);
    frame[4] = (uint8_t)(count >> 8);
    frame[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    uint8_t rx_buf[70];
    int rx_len = sendRaw(frame, 8, rx_buf, sizeof(rx_buf), timeout_ms);

    if (rx_len < 5) {
      snprintf(err_msg, err_size, "Timeout (recv %d B)", rx_len);
      return -1;
    }

    // Check exception response: slave | (func | 0x80) | exception_code | crc
    if (rx_buf[1] == (func | 0x80)) {
      snprintf(err_msg, err_size, "Modbus exception: 0x%02X", rx_buf[2]);
      return -1;
    }

    uint16_t calc_crc = calcCRC(rx_buf, rx_len - 2);
    uint16_t rx_crc = rx_buf[rx_len - 2] | (rx_buf[rx_len - 1] << 8);
    if (calc_crc != rx_crc) {
      snprintf(err_msg, err_size, "CRC err: calc 0x%04X != rx 0x%04X", calc_crc, rx_crc);
      return -1;
    }

    uint8_t byte_cnt = rx_buf[2];
    int reg_read = byte_cnt / 2;
    for (int i = 0; i < reg_read && i < (int)count; i++) {
      dest[i] = (rx_buf[3 + i * 2] << 8) | rx_buf[4 + i * 2];
    }
    return reg_read;
  }

  // Ghi 1 Holding Register (0x06)
  bool writeSingleRegister(uint8_t slave, uint16_t reg_addr, uint16_t value, char* err_msg, size_t err_size, uint32_t timeout_ms = 350) {
    uint8_t frame[8];
    frame[0] = slave;
    frame[1] = 0x06;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)(reg_addr & 0xFF);
    frame[4] = (uint8_t)(value >> 8);
    frame[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = calcCRC(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    uint8_t rx_buf[16];
    int rx_len = sendRaw(frame, 8, rx_buf, sizeof(rx_buf), timeout_ms);
    if (rx_len < 8) {
      snprintf(err_msg, err_size, "Timeout / short resp (%d B)", rx_len);
      return false;
    }
    uint16_t calc_crc = calcCRC(rx_buf, rx_len - 2);
    uint16_t rx_crc = rx_buf[rx_len - 2] | (rx_buf[rx_len - 1] << 8);
    if (calc_crc != rx_crc) {
      snprintf(err_msg, err_size, "CRC err");
      return false;
    }
    return true;
  }

private:
  HardwareSerial& _serial;
  int8_t _rx_pin;
  int8_t _tx_pin;
  int8_t _dir_pin;
};
