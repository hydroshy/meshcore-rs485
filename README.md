# RS485 Modbus Sensor Node (Standalone Project)

Dự án độc lập cho **Seeed Studio XIAO ESP32-C3** kết nối module **Wio SX1262 LoRa** và **TTL to RS485 (Auto-flow)**.

---

## 1. Cấu trúc thư mục

```text
sensor/RS485_SENSOR/
├── platformio.ini       # File cấu hình PlatformIO độc lập đầy đủ
├── src/
│   ├── main.cpp         # Chương trình chính: Mesh, CLI, Auto-send Group Chat
│   ├── RS485Modbus.h    # Driver Modbus RTU master
│   ├── target.cpp       # Khởi tạo phần cứng (Board, Radio SX1262, RTC Clock)
│   ├── target.h         # Khai báo extern
│   ├── XiaoC3Board.h    # Board abstraction cho Xiao C3
│   ├── SensorMesh.h     # Lớp nền MeshCore Sensor node
│   ├── SensorMesh.cpp   # Xử lý gói tin Sensor, CLI, ACL
│   ├── TimeSeriesData.h # Lưu trữ lịch sử dữ liệu
│   └── TimeSeriesData.cpp
├── include/
└── lib/
```

---

## 2. Sơ đồ nối chân

| Chân Module RS485 | Chân XIAO ESP32-C3 | Chú thích |
| :--- | :--- | :--- |
| **VCC** | **3V3** | Nguồn 3.3V |
| **GND** | **GND** | Nối đất |
| **TXD (DI)** | **D6 (GPIO21)** | UART1 TX |
| **RXD (RO)** | **D7 (GPIO20)** | UART1 RX |

*(Wio SX1262 LoRa sử dụng các chân D1, D2, D3, D4, D5, D8, D9, D10).*

---

## 3. Cách mở và biên dịch với PlatformIO (VS Code)

1. Mở VS Code.
2. Chọn **File -> Open Folder...** và chọn đúng thư mục:
   ```text
   sensor/RS485_SENSOR
   ```
3. PlatformIO sẽ tự nhận diện project `Xiao_C3_modbus_sensor`.
4. Bấm biểu tượng **Build (dấu tick)** ở thanh đáy hoặc mở terminal gõ:
   ```bash
   pio run
   ```
5. Cắm Xiao ESP32-C3 vào máy tính và bấm **Upload (mũi tên sang phải)**:
   ```bash
   pio run -t upload
   ```
6. Mở **Serial Monitor (115200 baud)**:
   ```bash
   pio device monitor
   ```

---

## 4. Các lệnh CLI điều khiển qua Serial / Mesh LoRa

- `mb help` : Xem trợ giúp lệnh Modbus.
- `mb cfg` : Xem cấu hình tự động gửi hiện tại.
- `mb send <giây>` : Đặt chu kỳ tự động đọc & gửi tin nhắn vào nhóm (0 = tắt). Ví dụ: `mb send 60`
- `mb chan <tên_kênh> [mật_khẩu]` : Đặt nhóm chat (kèm mật khẩu = nhóm riêng tư mã hóa). Ví dụ: `mb chan Farm1 123456`
- `mb auto <slave> <func:3|4> <reg> <count>` : Cấu hình thanh ghi Modbus đọc tự động. Ví dụ: `mb auto 1 3 0 2`
- `mb read <slave> <func:3|4> <reg> <count>` : Đọc thanh ghi ngay lập tức. Ví dụ: `mb read 1 3 0 2`
- `mb write <slave> <reg> <val>` : Ghi thanh ghi (Function 6).
- `mb raw <hex>` : Gửi chuỗi hex thô.
- `mb scan [start] [end]` : Dò ID slave Modbus.
