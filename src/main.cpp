#include <Arduino.h>
#include <SensorMesh.h>
#include <FS.h>
#include <SPIFFS.h>
#include "RS485Modbus.h"

#ifndef RS485_TX_PIN
  #define RS485_TX_PIN 21 // D6 / GPIO21 trên Seeed Xiao ESP32-C3
#endif
#ifndef RS485_RX_PIN
  #define RS485_RX_PIN 20 // D7 / GPIO20 trên Seeed Xiao ESP32-C3
#endif
#ifndef RS485_BAUD
  #define RS485_BAUD 9600
#endif

#define MB_CONFIG_FILE "/mb_cfg.bin"
#ifndef MAX_TEXT_LEN
  #define MAX_TEXT_LEN (10 * 16) // 160 bytes text payload theo chuẩn MeshCore
#endif

// Cấu hình định kỳ và nhóm chat (lưu Flash)
struct AutoSendConfig {
  uint32_t interval_sec;      // Chu kỳ gửi (giây), 0 = tắt
  uint8_t  slave_id;          // Modbus slave ID (1-247)
  uint8_t  func;              // Function code: 3 hoặc 4
  uint16_t reg_addr;          // Địa chỉ thanh ghi bắt đầu
  uint16_t reg_count;         // Số lượng thanh ghi đọc (1-10)
  char     channel_name[32];  // Tên nhóm chat (vd: "Farm_Private")
  uint8_t  channel_secret[32];// Khóa bí mật (PSK) để mã hóa nhóm private
  uint8_t  channel_hash[1];   // Hash định danh kênh
  bool     is_private;        // true = nhóm riêng mã hóa, false = public
};

static AutoSendConfig cfg;
RS485Modbus modbus(Serial1, RS485_RX_PIN, RS485_TX_PIN, -1);

// Khóa bí mật mặc định của kênh Public trong MeshCore (base64: "izOH6cXN6mrJ5e26oRXNcg==")
static const uint8_t MESHCORE_PUBLIC_PSK[16] = {
  0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
  0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
};
#define MESHCORE_PUBLIC_HASH 0x11

void loadConfig() {
  if (SPIFFS.exists(MB_CONFIG_FILE)) {
    File f = SPIFFS.open(MB_CONFIG_FILE, "r");
    if (f && f.size() == sizeof(AutoSendConfig)) {
      f.read((uint8_t*)&cfg, sizeof(AutoSendConfig));
      f.close();
      return;
    }
    if (f) f.close();
  }
  // Mặc định ban đầu: Kênh Public chuẩn của MeshCore
  memset(&cfg, 0, sizeof(AutoSendConfig));
  cfg.interval_sec = 0; // Mặc định tắt auto send
  cfg.slave_id = 1;
  cfg.func = 3;
  cfg.reg_addr = 0;
  cfg.reg_count = 2;
  strcpy(cfg.channel_name, "Public");
  cfg.is_private = false;
  cfg.channel_hash[0] = MESHCORE_PUBLIC_HASH;
  memcpy(cfg.channel_secret, MESHCORE_PUBLIC_PSK, 16);
}

void saveConfig() {
  File f = SPIFFS.open(MB_CONFIG_FILE, "w");
  if (f) {
    f.write((const uint8_t*)&cfg, sizeof(AutoSendConfig));
    f.close();
  }
}

class ModbusSensorNode : public SensorMesh {
public:
  ModbusSensorNode(mesh::MainBoard& b, mesh::Radio& r, mesh::MillisecondClock& ms,
                   mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& t)
    : SensorMesh(b, r, ms, rng, rtc, t), _next_auto_send(0) {}

  // Gửi tin nhắn Text vào Group Channel (Public hoặc Private)
  bool sendGroupTextMessage(const char* text) {
    mesh::GroupChannel grp;
    grp.hash[0] = cfg.channel_hash[0];
    memcpy(grp.secret, cfg.channel_secret, 32);

    uint8_t temp[5 + MAX_TEXT_LEN + 32];
    uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0; // TXT_TYPE_PLAIN

    int len = snprintf((char*)&temp[5], sizeof(temp) - 5, "%s: %s", getNodeName(), text);
    if (len < 0) return false;

    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, grp, temp, 5 + len);
    if (pkt) {
      sendFlood(pkt);
      return true;
    }
    return false;
  }

  void checkAndPerformAutoSend() {
    if (cfg.interval_sec == 0) return; // Đang tắt
    uint32_t now = millis();
    if (now < _next_auto_send) return;

    _next_auto_send = now + (cfg.interval_sec * 1000UL);

    // Đọc cảm biến Modbus
    uint16_t vals[10];
    char err[40];
    int n = modbus.readRegisters(cfg.slave_id, cfg.func, cfg.reg_addr, cfg.reg_count, vals, err, sizeof(err));

    char msg[120];
    if (n > 0) {
      char* p = msg;
      p += sprintf(p, "[MB ID:%u]", cfg.slave_id);
      for (int i = 0; i < n; i++) {
        p += sprintf(p, " R%u:%u", cfg.reg_addr + i, vals[i]);
      }
    } else {
      snprintf(msg, sizeof(msg), "[MB ID:%u] Read Err: %s", cfg.slave_id, err);
    }

    // Gửi vào nhóm chat
    sendGroupTextMessage(msg);
    Serial.printf("[AutoSend -> %s] %s\n", cfg.channel_name, msg);
  }

protected:
  uint32_t _next_auto_send;

  void onSensorDataRead() override {
    // Có thể đưa vào CayenneLPP Telemetry ở đây nếu có node truy vấn
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }

  // CLI Handler
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    while (*command == ' ') command++;

    // 1. mb help
    if (strcmp(command, "mb help") == 0 || strcmp(command, "mb") == 0) {
      strcpy(reply, "Cmds: mb read | mb write | mb raw | mb scan | mb send <sec> | mb chan <name> [pass] | mb auto <id> <fn> <reg> <cnt> | mb cfg");
      return true;
    }

    // 2. mb cfg - xem cấu hình hiện tại
    if (strcmp(command, "mb cfg") == 0) {
      snprintf(reply, 160, "AutoSend: %us | Group: '%s' (%s) | Target: ID %u, fn %u, reg %u, cnt %u",
               cfg.interval_sec, cfg.channel_name, cfg.is_private ? "Private" : "Public",
               cfg.slave_id, cfg.func, cfg.reg_addr, cfg.reg_count);
      return true;
    }

    // 3. mb send <interval_sec> - đặt chu kỳ gửi (0 để tắt)
    // Ví dụ: mb send 60 (gửi mỗi 1 phút), mb send 300 (mỗi 5 phút), mb send 0 (tắt)
    if (strncmp(command, "mb send ", 8) == 0) {
      uint32_t sec = atoi(command + 8);
      cfg.interval_sec = sec;
      saveConfig();
      _next_auto_send = millis() + (cfg.interval_sec * 1000UL);
      if (sec == 0) {
        strcpy(reply, "OK: AutoSend disabled");
      } else {
        snprintf(reply, 160, "OK: AutoSend set to %u seconds", sec);
      }
      return true;
    }

    // 4. mb chan <channel_name> [password]
    // Ví dụ đặt nhóm riêng mã hóa: mb chan FarmSecret 123456
    // Hoặc đặt nhóm public: mb chan Public
    if (strncmp(command, "mb chan ", 8) == 0) {
      char name[32] = {0};
      char pass[32] = {0};
      int parts = sscanf(command + 8, "%31s %31s", name, pass);
      if (parts < 1) {
        strcpy(reply, "Err: mb chan <name> [password]");
        return true;
      }
      strncpy(cfg.channel_name, name, sizeof(cfg.channel_name) - 1);

      if (parts >= 2 && strlen(pass) > 0) {
        cfg.is_private = true;
        memset(cfg.channel_secret, 0, 32);

        // Kiểm tra xem chuỗi có phải đúng 32 ký tự Hex (16 bytes key chuẩn của MeshCore App)
        bool is_hex = (strlen(pass) == 32);
        if (is_hex) {
          for (int i = 0; i < 32; i++) {
            if (!mesh::Utils::isHexChar(pass[i])) { is_hex = false; break; }
          }
        }

        if (is_hex) {
          mesh::Utils::fromHex(cfg.channel_secret, 16, pass);
          // Theo chuẩn MeshCore: Channel Hash = SHA256(16-byte secret)[0]
          mesh::Utils::sha256(cfg.channel_hash, sizeof(cfg.channel_hash), cfg.channel_secret, 16);
          snprintf(reply, 160, "OK: Channel '%s' key 16B hex (hash: 0x%02X)", name, cfg.channel_hash[0]);
        } else {
          // Nếu nhập mật khẩu text thông thường: hash sha256 thành key
          mesh::Utils::sha256(cfg.channel_secret, 16, (const uint8_t*)pass, strlen(pass));
          mesh::Utils::sha256(cfg.channel_hash, sizeof(cfg.channel_hash), cfg.channel_secret, 16);
          snprintf(reply, 160, "OK: Channel '%s' pass text (hash: 0x%02X)", name, cfg.channel_hash[0]);
        }
      } else {
        // Nhóm Public chuẩn của MeshCore
        cfg.is_private = false;
        cfg.channel_hash[0] = MESHCORE_PUBLIC_HASH;
        memset(cfg.channel_secret, 0, 32);
        memcpy(cfg.channel_secret, MESHCORE_PUBLIC_PSK, 16);
        snprintf(reply, 160, "OK: Channel '%s' set as PUBLIC", name);
      }
      saveConfig();
      return true;
    }

    // 5. mb auto <slave> <func> <reg> <count> - cấu hình thông số đọc định kỳ
    // Ví dụ: mb auto 1 3 0 2
    if (strncmp(command, "mb auto ", 8) == 0) {
      int slave, func, reg, count;
      if (sscanf(command + 8, "%d %d %d %d", &slave, &func, &reg, &count) != 4 || count < 1 || count > 10) {
        strcpy(reply, "Err: mb auto <slave> <func:3|4> <reg> <count(1-10)>");
        return true;
      }
      cfg.slave_id = slave;
      cfg.func = func;
      cfg.reg_addr = reg;
      cfg.reg_count = count;
      saveConfig();
      snprintf(reply, 160, "OK: Auto target set to ID:%d fn:%d reg:%d cnt:%d", slave, func, reg, count);
      return true;
    }

    // 6. mb read <slave> <func> <reg> <count>
    if (strncmp(command, "mb read ", 8) == 0) {
      int slave = 0, func = 3, reg = 0, count = 1;
      if (sscanf(command + 8, "%d %d %d %d", &slave, &func, &reg, &count) != 4) {
        strcpy(reply, "Err: mb read <slave> <func:3|4> <reg> <count>");
        return true;
      }
      uint16_t dest[32];
      char err[64] = {0};
      int n = modbus.readRegisters((uint8_t)slave, (uint8_t)func, (uint16_t)reg, (uint16_t)count, dest, err, sizeof(err));
      if (n < 0) {
        snprintf(reply, 160, "Fail: %s", err);
        return true;
      }
      char* p = reply;
      p += sprintf(p, "OK [%d]:", n);
      for (int i = 0; i < n && (p - reply) < 140; i++) {
        p += sprintf(p, " %u", dest[i]);
      }
      return true;
    }

    // 7. mb write <slave> <reg> <val>
    if (strncmp(command, "mb write ", 9) == 0) {
      int slave = 0, reg = 0, val = 0;
      if (sscanf(command + 9, "%d %d %d", &slave, &reg, &val) != 3) {
        strcpy(reply, "Err: mb write <slave> <reg> <val>");
        return true;
      }
      char err[64] = {0};
      if (modbus.writeSingleRegister((uint8_t)slave, (uint16_t)reg, (uint16_t)val, err, sizeof(err))) {
        snprintf(reply, 160, "OK: Wrote %u to reg %u", (uint16_t)val, (uint16_t)reg);
      } else {
        snprintf(reply, 160, "Fail: %s", err);
      }
      return true;
    }

    // 8. mb raw <hex>
    if (strncmp(command, "mb raw ", 7) == 0) {
      const char* hex = command + 7;
      int hex_len = strlen(hex);
      if (hex_len % 2 != 0 || hex_len > 40) {
        strcpy(reply, "Err: Hex length must be even and <= 40");
        return true;
      }
      uint8_t tx_buf[20];
      int send_len = hex_len / 2;
      mesh::Utils::fromHex(tx_buf, send_len, hex);

      uint8_t rx_buf[32];
      int rx_cnt = modbus.sendRaw(tx_buf, send_len, rx_buf, sizeof(rx_buf), 400);
      if (rx_cnt == 0) {
        strcpy(reply, "Timeout: no response");
        return true;
      }
      char* p = reply;
      p += sprintf(p, "RX[%d]: ", rx_cnt);
      for (int i = 0; i < rx_cnt && (p - reply) < 150; i++) {
        p += sprintf(p, "%02X", rx_buf[i]);
      }
      return true;
    }

    // 9. mb scan [start] [end]
    if (strncmp(command, "mb scan", 7) == 0) {
      int start_id = 1;
      int end_id = 16;
      sscanf(command + 7, "%d %d", &start_id, &end_id);
      if (start_id < 1) start_id = 1;
      if (end_id > 247) end_id = 247;

      char* p = reply;
      p += sprintf(p, "Found:");
      int found_cnt = 0;
      for (int id = start_id; id <= end_id; id++) {
        uint16_t dummy[1];
        char err[32];
        if (modbus.readRegisters((uint8_t)id, 3, 0, 1, dummy, err, sizeof(err), 100) >= 0 ||
            modbus.readRegisters((uint8_t)id, 4, 0, 1, dummy, err, sizeof(err), 100) >= 0) {
          p += sprintf(p, " %d", id);
          found_cnt++;
        }
      }
      if (found_cnt == 0) {
        strcpy(reply, "No Modbus slave found in range");
      }
      return true;
    }

    return false;
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
ModbusSensorNode the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() { while (1); }
static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  modbus.begin(RS485_BAUD);

  board.begin();
  if (!radio_init()) halt();
  fast_rng.begin(radio_driver.getRngSeed());

  SPIFFS.begin(true);
  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();
    store.save("_main", the_mesh.self_id);
  }

  loadConfig(); // Nạp cấu hình Modbus định kỳ từ Flash

  sensors.begin();
  the_mesh.begin(&SPIFFS);

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  Serial.println("\n[MeshCore] Xiao ESP32-C3 RS485 Modbus Sensor Node Ready.");
  Serial.println("Type 'mb help' for Modbus CLI commands.");
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < (int)sizeof(command) - 1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len > 0 && command[len - 1] == '\r') {
    command[len - 1] = 0;
    char reply[160] = {0};
    the_mesh.handleCommand(NULL, 0, command, reply);
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }
    command[0] = 0;
  }

  the_mesh.loop();
  the_mesh.checkAndPerformAutoSend(); // Định kỳ tự đọc Modbus và bắn tin nhắn vào Group Chat
}
