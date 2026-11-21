#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <FS.h>
#include <SPIFFS.h>
#include "RS485Modbus.h"

#define MB_CONFIG_FILE "/mb_cfg_v2.bin"
#define MAX_MB_CMDS 16

#ifndef MAX_TEXT_LEN
  #define MAX_TEXT_LEN (10 * 16)
#endif

// Khóa bí mật mặc định của kênh Public trong MeshCore (base64: "izOH6cXN6mrJ5e26oRXNcg==")
static const uint8_t MESHCORE_PUBLIC_PSK[16] = {
  0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
  0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
};
#define MESHCORE_PUBLIC_HASH 0x11

// Kiểu dữ liệu hỗ trợ
enum ModbusDataType : uint8_t {
  MB_TYPE_U16 = 0,
  MB_TYPE_S16 = 1,
  MB_TYPE_U32 = 2,
  MB_TYPE_S32 = 3,
  MB_TYPE_FLOAT = 4,
  MB_TYPE_HEX = 5,
  MB_TYPE_STRING = 6
};

// Thứ tự byte / word
enum ModbusEndian : uint8_t {
  MB_ENDIAN_ABCD = 0, // Big-Endian
  MB_ENDIAN_CDAB = 1, // Word-Swapped (Selec EM2M chuẩn)
  MB_ENDIAN_BADC = 2, // Byte-Swapped
  MB_ENDIAN_DCBA = 3  // Little-Endian
};

// Cấu hình một lệnh đọc Modbus (cmd_id)
struct ModbusCmdItem {
  uint8_t  enabled;      // 0 = trống, 1 = kích hoạt
  uint8_t  slave_id;     // 1-247
  uint8_t  func;         // 3 hoặc 4
  uint16_t reg_addr;     // Địa chỉ thanh ghi
  uint8_t  reg_length;   // Số lượng thanh ghi (1-10)
  uint8_t  data_type;    // ModbusDataType (0:u16, 1:s16, 2:u32, 3:s32, 4:float)
  uint8_t  endian;       // ModbusEndian (0:abcd, 1:cdab, 2:badc, 3:dcba)
};

// Cấu hình toàn bộ hệ thống Modbus
struct ModbusSystemConfig {
  uint32_t cycle_sec;             // Chu kỳ toàn bộ một vòng quét (mặc định 120s)
  uint32_t cycle_dg_sec;          // Khoảng cách giữa các datagram (mặc định 20s)
  char     channel_name[32];      // Tên nhóm chat (vd: "Public", "mb485")
  uint8_t  channel_secret[32];    // Khóa bí mật
  uint8_t  channel_hash[1];       // Hash kênh
  uint8_t  is_private;            // 0: Public, 1: Private
  ModbusCmdItem cmds[MAX_MB_CMDS];// Bảng 16 lệnh đọc
};

class ModbusCommandHandler {
public:
  ModbusSystemConfig cfg;

  ModbusCommandHandler() : _current_cmd_idx(0), _is_running_cycle(false),
                           _next_dg_time(0), _next_cycle_start(0) {}

  void loadConfig() {
    if (SPIFFS.exists(MB_CONFIG_FILE)) {
      File f = SPIFFS.open(MB_CONFIG_FILE, "r");
      if (f && f.size() == sizeof(ModbusSystemConfig)) {
        f.read((uint8_t*)&cfg, sizeof(ModbusSystemConfig));
        f.close();
        return;
      }
      if (f) f.close();
    }

    // Cấu hình mặc định
    memset(&cfg, 0, sizeof(ModbusSystemConfig));
    cfg.cycle_sec = 120;     // Mặc định 2 phút
    cfg.cycle_dg_sec = 20;   // Mặc định 20 giây giữa các datagram
    strcpy(cfg.channel_name, "Public");
    cfg.is_private = 0;
    cfg.channel_hash[0] = MESHCORE_PUBLIC_HASH;
    memcpy(cfg.channel_secret, MESHCORE_PUBLIC_PSK, 16);
  }

  void saveConfig() {
    File f = SPIFFS.open(MB_CONFIG_FILE, "w");
    if (f) {
      f.write((const uint8_t*)&cfg, sizeof(ModbusSystemConfig));
      f.close();
    }
  }

  // Đếm số lượng lệnh cmd_id đang được kích hoạt
  int getActiveCmdCount() const {
    int count = 0;
    for (int i = 0; i < MAX_MB_CMDS; i++) {
      if (cfg.cmds[i].enabled) count++;
    }
    return count;
  }

  // Tính toán thời gian tối thiểu của 1 cycle dựa trên số cmd và cycle_dg
  uint32_t getMinSafeCycle() const {
    int active = getActiveCmdCount();
    if (active <= 1) return cfg.cycle_dg_sec;
    return (uint32_t)active * cfg.cycle_dg_sec;
  }

  // Helper chuyển đổi chuỗi sang Data Type
  static bool parseDataType(const char* str, uint8_t& type) {
    if (strcasecmp(str, "u16") == 0) { type = MB_TYPE_U16; return true; }
    if (strcasecmp(str, "s16") == 0) { type = MB_TYPE_S16; return true; }
    if (strcasecmp(str, "u32") == 0) { type = MB_TYPE_U32; return true; }
    if (strcasecmp(str, "s32") == 0) { type = MB_TYPE_S32; return true; }
    if (strcasecmp(str, "float") == 0) { type = MB_TYPE_FLOAT; return true; }
    if (strcasecmp(str, "hex") == 0) { type = MB_TYPE_HEX; return true; }
    if (strcasecmp(str, "string") == 0 || strcasecmp(str, "str") == 0) { type = MB_TYPE_STRING; return true; }
    return false;
  }

  static const char* getDataTypeName(uint8_t type) {
    switch (type) {
      case MB_TYPE_U16: return "u16";
      case MB_TYPE_S16: return "s16";
      case MB_TYPE_U32: return "u32";
      case MB_TYPE_S32: return "s32";
      case MB_TYPE_FLOAT: return "float";
      case MB_TYPE_HEX: return "hex";
      case MB_TYPE_STRING: return "string";
      default: return "raw";
    }
  }

  // Helper chuyển đổi chuỗi sang Endian
  static bool parseEndian(const char* str, uint8_t& endian) {
    if (strcasecmp(str, "abcd") == 0) { endian = MB_ENDIAN_ABCD; return true; }
    if (strcasecmp(str, "cdab") == 0) { endian = MB_ENDIAN_CDAB; return true; }
    if (strcasecmp(str, "badc") == 0) { endian = MB_ENDIAN_BADC; return true; }
    if (strcasecmp(str, "dcba") == 0) { endian = MB_ENDIAN_DCBA; return true; }
    return false;
  }

  static const char* getEndianName(uint8_t endian) {
    switch (endian) {
      case MB_ENDIAN_ABCD: return "abcd";
      case MB_ENDIAN_CDAB: return "cdab";
      case MB_ENDIAN_BADC: return "badc";
      case MB_ENDIAN_DCBA: return "dcba";
      default: return "abcd";
    }
  }

  // Đổi thứ tự byte/word theo endian cấu hình
  static uint32_t reorder32(uint16_t w0, uint16_t w1, uint8_t endian) {
    uint8_t b[4];
    b[0] = (uint8_t)(w0 >> 8);   // A
    b[1] = (uint8_t)(w0 & 0xFF); // B
    b[2] = (uint8_t)(w1 >> 8);   // C
    b[3] = (uint8_t)(w1 & 0xFF); // D

    uint8_t out[4];
    switch (endian) {
      case MB_ENDIAN_ABCD: // Standard Big Endian: A B C D
        out[0] = b[0]; out[1] = b[1]; out[2] = b[2]; out[3] = b[3];
        break;
      case MB_ENDIAN_CDAB: // Word Swap: C D A B (Selec EM2M)
        out[0] = b[2]; out[1] = b[3]; out[2] = b[0]; out[3] = b[1];
        break;
      case MB_ENDIAN_BADC: // Byte Swap: B A D C
        out[0] = b[1]; out[1] = b[0]; out[2] = b[3]; out[3] = b[2];
        break;
      case MB_ENDIAN_DCBA: // Little Endian: D C B A
        out[0] = b[3]; out[1] = b[2]; out[2] = b[1]; out[3] = b[0];
        break;
    }
    return ((uint32_t)out[0] << 24) | ((uint32_t)out[1] << 16) | ((uint32_t)out[2] << 8) | out[3];
  }

  // Tối ưu hóa chuỗi hiển thị giá trị đọc được (Gọn gàng, tiết kiệm payload LoRa & RAM)
  void formatValueString(const ModbusCmdItem& item, const uint16_t* raw_words, int words_read, char* dest, size_t dest_size) {
    if (words_read <= 0) {
      snprintf(dest, dest_size, "NaN");
      return;
    }

    if (item.data_type == MB_TYPE_HEX) {
      char* p = dest;
      for (int i = 0; i < words_read && (dest_size - (p - dest)) > 5; i++) {
        p += snprintf(p, dest_size - (p - dest), "%04X", raw_words[i]);
      }
    } else if (item.data_type == MB_TYPE_STRING) {
      // Modbus ASCII string: mỗi thanh ghi 16-bit chứa 2 ký tự ASCII
      char* p = dest;
      for (int i = 0; i < words_read && (dest_size - (p - dest)) > 3; i++) {
        char c1, c2;
        if (item.endian == MB_ENDIAN_BADC || item.endian == MB_ENDIAN_DCBA) {
          c1 = (char)(raw_words[i] & 0xFF);
          c2 = (char)(raw_words[i] >> 8);
        } else {
          c1 = (char)(raw_words[i] >> 8);
          c2 = (char)(raw_words[i] & 0xFF);
        }
        if (c1 >= 32 && c1 <= 126) *p++ = c1;
        if (c2 >= 32 && c2 <= 126) *p++ = c2;
      }
      *p = '\0';
      if (dest[0] == '\0') snprintf(dest, dest_size, "empty");
    } else if (item.data_type == MB_TYPE_FLOAT && words_read >= 2) {
      uint32_t raw = reorder32(raw_words[0], raw_words[1], item.endian);
      float val = 0;
      memcpy(&val, &raw, 4);
      snprintf(dest, dest_size, "%.2f", val);
    } else if (item.data_type == MB_TYPE_U32 && words_read >= 2) {
      uint32_t val = reorder32(raw_words[0], raw_words[1], item.endian);
      snprintf(dest, dest_size, "%u", val);
    } else if (item.data_type == MB_TYPE_S32 && words_read >= 2) {
      int32_t val = (int32_t)reorder32(raw_words[0], raw_words[1], item.endian);
      snprintf(dest, dest_size, "%d", val);
    } else if (item.data_type == MB_TYPE_S16) {
      snprintf(dest, dest_size, "%d", (int16_t)raw_words[0]);
    } else {
      // Mặc định U16
      snprintf(dest, dest_size, "%u", raw_words[0]);
    }
  }

  // Xử lý toàn bộ lệnh CLI Modbus (tách rời khỏi main.cpp)
  bool handleCLI(char* command, char* reply, size_t reply_size, RS485Modbus& modbus) {
    while (*command == ' ') command++;

    // 1. mb help
    if (strcmp(command, "mb help") == 0 || strcmp(command, "mb") == 0) {
      snprintf(reply, reply_size,
        "CLI:\n"
        "  mb cmd <id(0-15)> <slave> <func:3|4> <reg> <len> <type:u16|s16|u32|s32|float|hex|string> <endian:abcd|cdab|badc|dcba>\n"
        "  mb cmd del <id> | mb cmd list\n"
        "  mb cycle <sec> | mb cycle_dg <sec> | mb chan <name> [hex_key/pass]\n"
        "  mb read <slave> <func> <reg> <len> | mb write <slave> <reg> <val> | mb scan [s e]");
      return true;
    }

    // 2. mb cmd list
    if (strcmp(command, "mb cmd list") == 0 || strcmp(command, "mb list") == 0) {
      char* p = reply;
      int active = getActiveCmdCount();
      p += snprintf(p, reply_size - (p - reply), "Cycle:%us Dg:%us (MinReq:%us) Active:%d/%d\n",
                    cfg.cycle_sec, cfg.cycle_dg_sec, getMinSafeCycle(), active, MAX_MB_CMDS);

      for (int i = 0; i < MAX_MB_CMDS; i++) {
        if (!cfg.cmds[i].enabled) continue;
        const auto& c = cfg.cmds[i];
        p += snprintf(p, reply_size - (p - reply), "#%d S:%d F:%d R:%d L:%d %s %s\n",
                      i, c.slave_id, c.func, c.reg_addr, c.reg_length,
                      getDataTypeName(c.data_type), getEndianName(c.endian));
        if ((reply_size - (p - reply)) < 30) break;
      }
      return true;
    }

    // 3. mb cmd del <id>
    if (strncmp(command, "mb cmd del ", 11) == 0) {
      int id = atoi(command + 11);
      if (id >= 0 && id < MAX_MB_CMDS) {
        cfg.cmds[id].enabled = 0;
        saveConfig();
        snprintf(reply, reply_size, "OK: Cmd #%d deleted", id);
      } else {
        snprintf(reply, reply_size, "Err: id must be 0-%d", MAX_MB_CMDS - 1);
      }
      return true;
    }

    // 4. mb cmd <id> <slave> <func> <reg> <reg_length> <data_type> <endian>
    // Ví dụ Selec EM2M Điện áp: mb cmd 0 1 4 20 2 float cdab
    if (strncmp(command, "mb cmd ", 7) == 0) {
      int id = -1, slave = 1, func = 4, reg = 0, len = 2;
      char type_str[16] = "float";
      char endian_str[16] = "cdab";

      int parts = sscanf(command + 7, "%d %d %d %d %d %15s %15s",
                         &id, &slave, &func, &reg, &len, type_str, endian_str);
      if (parts < 5) {
        snprintf(reply, reply_size, "Err: mb cmd <id(0-%d)> <slave> <func> <reg> <len> [type] [endian]", MAX_MB_CMDS - 1);
        return true;
      }

      if (id < 0 || id >= MAX_MB_CMDS) {
        snprintf(reply, reply_size, "Err: id must be 0-%d", MAX_MB_CMDS - 1);
        return true;
      }

      uint8_t dt = MB_TYPE_FLOAT;
      uint8_t ed = MB_ENDIAN_CDAB;
      if (parts >= 6 && !parseDataType(type_str, dt)) {
        snprintf(reply, reply_size, "Err: type must be u16|s16|u32|s32|float|hex|string");
        return true;
      }
      if (parts >= 7 && !parseEndian(endian_str, ed)) {
        snprintf(reply, reply_size, "Err: endian must be abcd|cdab|badc|dcba");
        return true;
      }

      cfg.cmds[id].enabled = 1;
      cfg.cmds[id].slave_id = (uint8_t)slave;
      cfg.cmds[id].func = (uint8_t)func;
      cfg.cmds[id].reg_addr = (uint16_t)reg;
      cfg.cmds[id].reg_length = (uint8_t)len;
      cfg.cmds[id].data_type = dt;
      cfg.cmds[id].endian = ed;
      saveConfig();

      // Kiểm tra tính hợp lệ với cycle
      uint32_t min_req = getMinSafeCycle();
      if (cfg.cycle_sec > 0 && cfg.cycle_sec < min_req) {
        snprintf(reply, reply_size, "OK: Added #%d (WARN: cycle %us < min required %us (%d cmds * %us))",
                 id, cfg.cycle_sec, min_req, getActiveCmdCount(), cfg.cycle_dg_sec);
      } else {
        snprintf(reply, reply_size, "OK: Added Cmd #%d [S:%d F:%d R:%d L:%d %s %s]",
                 id, slave, func, reg, len, getDataTypeName(dt), getEndianName(ed));
      }
      return true;
    }

    // 5. mb cycle_dg <sec> - Khoảng thời gian giữa các datagram
    if (strncmp(command, "mb cycle_dg ", 12) == 0) {
      uint32_t sec = atoi(command + 12);
      if (sec < 3 || sec > 3600) {
        snprintf(reply, reply_size, "Err: cycle_dg must be 3-3600s");
        return true;
      }
      cfg.cycle_dg_sec = sec;
      saveConfig();

      uint32_t min_req = getMinSafeCycle();
      if (cfg.cycle_sec > 0 && cfg.cycle_sec < min_req) {
        snprintf(reply, reply_size, "OK: Datagram interval = %us (WARN: cycle %us < min %us)", sec, cfg.cycle_sec, min_req);
      } else {
        snprintf(reply, reply_size, "OK: Datagram interval = %us", sec);
      }
      return true;
    }

    // 6. mb cycle <sec> - Chu kỳ quét toàn bộ các lệnh cmd_id
    if (strncmp(command, "mb cycle ", 9) == 0) {
      uint32_t sec = atoi(command + 9);
      if (sec > 0) {
        uint32_t min_req = getMinSafeCycle();
        if (sec < min_req) {
          snprintf(reply, reply_size, "Err: cycle %us too short! Need at least %us (%d active cmds * %us datagram_cycle)",
                   sec, min_req, getActiveCmdCount(), cfg.cycle_dg_sec);
          return true;
        }
      }
      cfg.cycle_sec = sec;
      saveConfig();
      if (sec == 0) {
        snprintf(reply, reply_size, "OK: Cycle polling disabled");
      } else {
        snprintf(reply, reply_size, "OK: Polling cycle = %us (%u min)", sec, sec / 60);
      }
      return true;
    }

    // 7. mb chan <name> [key]
    if (strncmp(command, "mb chan ", 8) == 0) {
      char name[32] = {0};
      char pass[64] = {0};
      int parts = sscanf(command + 8, "%31s %63s", name, pass);
      if (parts < 1) {
        snprintf(reply, reply_size, "Err: mb chan <name> [key]");
        return true;
      }
      strncpy(cfg.channel_name, name, sizeof(cfg.channel_name) - 1);

      if (parts >= 2 && strlen(pass) > 0) {
        cfg.is_private = 1;
        memset(cfg.channel_secret, 0, 32);

        bool is_hex = (strlen(pass) == 32);
        if (is_hex) {
          for (int i = 0; i < 32; i++) {
            if (!mesh::Utils::isHexChar(pass[i])) { is_hex = false; break; }
          }
        }

        if (is_hex) {
          mesh::Utils::fromHex(cfg.channel_secret, 16, pass);
          mesh::Utils::sha256(cfg.channel_hash, sizeof(cfg.channel_hash), cfg.channel_secret, 16);
          snprintf(reply, reply_size, "OK: Channel '%s' key 16B hex (hash: 0x%02X)", name, cfg.channel_hash[0]);
        } else {
          mesh::Utils::sha256(cfg.channel_secret, 16, (const uint8_t*)pass, strlen(pass));
          mesh::Utils::sha256(cfg.channel_hash, sizeof(cfg.channel_hash), cfg.channel_secret, 16);
          snprintf(reply, reply_size, "OK: Channel '%s' pass text (hash: 0x%02X)", name, cfg.channel_hash[0]);
        }
      } else {
        cfg.is_private = 0;
        cfg.channel_hash[0] = MESHCORE_PUBLIC_HASH;
        memset(cfg.channel_secret, 0, 32);
        memcpy(cfg.channel_secret, MESHCORE_PUBLIC_PSK, 16);
        snprintf(reply, reply_size, "OK: Channel '%s' set as PUBLIC", name);
      }
      saveConfig();
      return true;
    }

    // 8. mb read <slave> <func> <reg> <len> (Đọc thủ công)
    if (strncmp(command, "mb read ", 8) == 0) {
      int slave = 1, func = 3, reg = 0, count = 1;
      if (sscanf(command + 8, "%d %d %d %d", &slave, &func, &reg, &count) != 4) {
        snprintf(reply, reply_size, "Err: mb read <slave> <func:3|4> <reg> <count>");
        return true;
      }
      uint16_t dest[32];
      char err[64] = {0};
      int n = modbus.readRegisters((uint8_t)slave, (uint8_t)func, (uint16_t)reg, (uint16_t)count, dest, err, sizeof(err));
      if (n < 0) {
        snprintf(reply, reply_size, "Fail: %s", err);
        return true;
      }
      char* p = reply;
      p += snprintf(p, reply_size, "OK [%d]:", n);
      for (int i = 0; i < n && (p - reply) < 100; i++) {
        p += snprintf(p, reply_size - (p - reply), " %u", dest[i]);
      }
      if (n >= 2) {
        uint32_t raw_cdab = reorder32(dest[0], dest[1], MB_ENDIAN_CDAB);
        float val_float = 0;
        memcpy(&val_float, &raw_cdab, 4);
        p += snprintf(p, reply_size - (p - reply), " | Float: %.2f", val_float);
      }
      return true;
    }

    // 9. mb write <slave> <reg> <val>
    if (strncmp(command, "mb write ", 9) == 0) {
      int slave = 1, reg = 0, val = 0;
      if (sscanf(command + 9, "%d %d %d", &slave, &reg, &val) != 3) {
        snprintf(reply, reply_size, "Err: mb write <slave> <reg> <val>");
        return true;
      }
      char err[64] = {0};
      if (modbus.writeSingleRegister((uint8_t)slave, (uint16_t)reg, (uint16_t)val, err, sizeof(err))) {
        snprintf(reply, reply_size, "OK: Wrote %u to reg %u", (uint16_t)val, (uint16_t)reg);
      } else {
        snprintf(reply, reply_size, "Fail: %s", err);
      }
      return true;
    }

    // 10. mb scan [start] [end]
    if (strncmp(command, "mb scan", 7) == 0) {
      int start_id = 1, end_id = 16;
      sscanf(command + 7, "%d %d", &start_id, &end_id);
      if (start_id < 1) start_id = 1;
      if (end_id > 247) end_id = 247;

      char* p = reply;
      p += snprintf(p, reply_size, "Found:");
      int found_cnt = 0;
      for (int id = start_id; id <= end_id; id++) {
        uint16_t dummy[1];
        char err[32];
        if (modbus.readRegisters((uint8_t)id, 3, 0, 1, dummy, err, sizeof(err), 100) >= 0 ||
            modbus.readRegisters((uint8_t)id, 4, 0, 1, dummy, err, sizeof(err), 100) >= 0) {
          p += snprintf(p, reply_size - (p - reply), " %d", id);
          found_cnt++;
        }
      }
      if (found_cnt == 0) snprintf(reply, reply_size, "No Modbus slave found in range");
      return true;
    }

    return false;
  }

  // Máy trạng thái lập lịch quét cycle & datagram
  // Trả về true nếu có datagram cần gửi, lưu nội dung vào out_msg
  bool processScheduler(uint32_t now_ms, RS485Modbus& modbus, char* out_msg, size_t out_size) {
    if (cfg.cycle_sec == 0 || getActiveCmdCount() == 0) return false;

    // Nếu chưa trong đợt phát chu kỳ: kiểm tra xem đã đến giờ bắt đầu chu kỳ mới chưa
    if (!_is_running_cycle) {
      if (now_ms >= _next_cycle_start) {
        _is_running_cycle = true;
        _current_cmd_idx = 0;
        _next_dg_time = now_ms; // Phát lệnh đầu tiên ngay lập tức
        _next_cycle_start = now_ms + (cfg.cycle_sec * 1000UL);
      } else {
        return false;
      }
    }

    // Đang trong tiến trình phát lần lượt từng datagram
    if (now_ms < _next_dg_time) return false;

    // Tìm lệnh cmd_id tiếp theo đang được kích hoạt
    while (_current_cmd_idx < MAX_MB_CMDS && !cfg.cmds[_current_cmd_idx].enabled) {
      _current_cmd_idx++;
    }

    // Nếu đã duyệt hết tất cả các lệnh trong chu kỳ này
    if (_current_cmd_idx >= MAX_MB_CMDS) {
      _is_running_cycle = false;
      _current_cmd_idx = 0;
      return false;
    }

    // Tiến hành đọc Modbus cho lệnh này
    const auto& cmd = cfg.cmds[_current_cmd_idx];
    uint16_t raw_words[16];
    char err_buf[40];
    int n = modbus.readRegisters(cmd.slave_id, cmd.func, cmd.reg_addr, cmd.reg_length, raw_words, err_buf, sizeof(err_buf));

    char val_str[36];
    if (n > 0) {
      formatValueString(cmd, raw_words, n, val_str, sizeof(val_str));
      // Tối ưu hóa cực hạn: Loại bỏ [S:.. R:..], chỉ giữ mã lệnh #<id> và giá trị
      snprintf(out_msg, out_size, "#%d: %s", _current_cmd_idx, val_str);
    } else {
      snprintf(out_msg, out_size, "#%d: Err", _current_cmd_idx);
    }

    // Lên lịch phát cho datagram tiếp theo
    _current_cmd_idx++;
    _next_dg_time = now_ms + (cfg.cycle_dg_sec * 1000UL);
    return true;
  }

private:
  int      _current_cmd_idx;
  bool     _is_running_cycle;
  uint32_t _next_dg_time;
  uint32_t _next_cycle_start;
};
