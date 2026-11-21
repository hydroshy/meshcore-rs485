#include <Arduino.h>
#include <SensorMesh.h>
#include <FS.h>
#include <SPIFFS.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "RS485Modbus.h"
#include "ModbusCommandHandler.h"

#ifndef RS485_TX_PIN
  #define RS485_TX_PIN 21 // D6 / GPIO21 trên Seeed Xiao ESP32-C3
#endif
#ifndef RS485_RX_PIN
  #define RS485_RX_PIN 20 // D7 / GPIO20 trên Seeed Xiao ESP32-C3
#endif
#ifndef RS485_BAUD
  #define RS485_BAUD 9600
#endif

RS485Modbus modbus(Serial1, RS485_RX_PIN, RS485_TX_PIN, -1);
ModbusCommandHandler mb_mgr;

class ModbusSensorNode : public SensorMesh {
public:
  ModbusSensorNode(mesh::MainBoard& b, mesh::Radio& r, mesh::MillisecondClock& ms,
                   mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& t)
    : SensorMesh(b, r, ms, rng, rtc, t) {}

  // Gửi tin nhắn Text tối ưu vào Group Channel (Public hoặc Private)
  bool sendGroupTextMessage(const char* text) {
    mesh::GroupChannel grp;
    grp.hash[0] = mb_mgr.cfg.channel_hash[0];
    memcpy(grp.secret, mb_mgr.cfg.channel_secret, 32);

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
    char datagram_msg[80];
    if (mb_mgr.processScheduler(millis(), modbus, datagram_msg, sizeof(datagram_msg))) {
      sendGroupTextMessage(datagram_msg);
      Serial.printf("[TX -> %s] %s\n", mb_mgr.cfg.channel_name, datagram_msg);
    }
  }

protected:
  void onSensorDataRead() override {
    // Telemetry readback hook
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }

  // CLI Handler ủy quyền cho ModbusCommandHandler xử lý
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    return mb_mgr.handleCLI(command, reply, 160, modbus);
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
ModbusSensorNode the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() { while (1); }
static char command[160];

void setup() {
  // 1. Tối ưu năng lượng: Tắt hoàn toàn radio WiFi và Bluetooth BLE của ESP32-C3
  esp_wifi_stop();
  esp_wifi_deinit();
  #if defined(CONFIG_BT_ENABLED)
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
  #endif

  Serial.begin(115200);
  delay(1000);

  board.begin();
  if (!radio_init()) halt();
  fast_rng.begin(radio_driver.getRngSeed());

  // Mount SPIFFS trước khi đọc/ghi file
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  }

  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();
    store.save("_main", the_mesh.self_id);
  }

  // Khởi tạo và nạp cấu hình lệnh Modbus từ Flash
  mb_mgr.loadConfig();

  the_mesh.begin(&SPIFFS);

  // Khởi động UART1 RS485 ở cuối setup() để đảm bảo độc quyền D6/D7
  modbus.begin(RS485_BAUD);

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  Serial.println("\n[MeshCore] Xiao ESP32-C3 Modbus Multi-Command Sensor Ready.");
  Serial.println("Type 'mb help' or 'mb list' for command list.");
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
  the_mesh.checkAndPerformAutoSend();
}
