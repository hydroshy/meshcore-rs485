# Node Sensor RS485 Modbus - Seeed Studio Xiao ESP32-C3 & Wio-SX1262

Firmware sensor node chạy trên nền tảng MeshCore:
- Giao tiếp cảm biến Modbus RTU qua module TTL-to-RS485 (chỉ cần 2 dây TX/RX).
- **Tự động đọc theo chu kỳ và bắn tin nhắn vào Group Chat (kênh chung hoặc kênh riêng tư được mã hóa)**.
- Toàn bộ tham số cấu hình được lưu vào SPIFFS flash, mất điện không mất cấu hình.

---

## 1. Sơ đồ nối dây

| Module TTL to RS485 (Auto Flow) | Xiao ESP32-C3 | Chú thích |
| :--- | :--- | :--- |
| **VCC** | **3V3** (hoặc 5V) | Cấp nguồn |
| **GND** | **GND** | Mass chung |
| **TXD (DI)** | **D6 (GPIO21)** | Hardware Serial1 TX của ESP32-C3 |
| **RXD (RO)** | **D7 (GPIO20)** | Hardware Serial1 RX của ESP32-C3 |

---

## 2. Bảng lệnh CLI cấu hình định kỳ & nhóm chat

Gõ lệnh qua cổng Serial USB (115200 baud) hoặc gửi qua LoRa Mesh từ xa:

### A. Cấu hình gửi chu kỳ & Nhóm chat

| Lệnh | Ý nghĩa | Ví dụ |
| :--- | :--- | :--- |
| `mb cfg` | Xem toàn bộ cấu hình tự động hiện tại | `mb cfg` |
| `mb send <giây>` | Đặt chu kỳ tự động đọc & gửi tin nhắn (`0` = tắt) | `mb send 60` (mỗi 1 phút), `mb send 300` (mỗi 5 phút) |
| `mb chan <tên_kênh> [mật_khẩu]` | Cấu hình nhóm chat đích. Nếu có mật khẩu sẽ tự mã hóa (Private Channel) | `mb chan Farm1 secret123` (Nhóm riêng)<br>`mb chan Public` (Nhóm chung) |
| `mb auto <slave> <func> <reg> <count>` | Cấu hình cảm biến Modbus cần đọc tự động | `mb auto 1 3 0 2` (Đọc Slave 1, fn 3, reg 0, 2 thanh ghi) |

> **Format tin nhắn gửi vào nhóm chat:**  
> `XiaoC3 Modbus: [MB ID:1] R0:256 R1:600`  
> Tất cả các máy/điện thoại tham gia cùng nhóm chat sẽ thấy tin nhắn này theo chu kỳ.

### B. Lệnh Modbus thủ công

| Lệnh | Ý nghĩa | Ví dụ |
| :--- | :--- | :--- |
| `mb read <id> <fn> <reg> <cnt>` | Đọc thanh ghi ngay lập tức | `mb read 1 3 0 2` |
| `mb write <id> <reg> <val>` | Ghi 1 thanh ghi (Function 6) | `mb write 1 10 250` |
| `mb raw <hex>` | Gửi chuỗi Hex thô và nhận phản hồi | `mb raw 010300000002C40B` |
| `mb scan [start] [end]` | Dò tìm Slave ID trên đường dây RS485 | `mb scan 1 16` |

---

## 3. Cách nạp firmware với PlatformIO

1. Thêm cấu hình trong `platformio_snippet.ini` vào file `MeshCore/variants/xiao_c3/platformio.ini`.
2. Mở terminal tại thư mục `MeshCore`:
   ```bash
   pio run -e Xiao_C3_modbus_sensor -t upload
   ```
3. Mở Serial Monitor:
   ```bash
   pio device monitor -b 115200
   ```
