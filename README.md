#  Micromouse Fast-Runner

> Dự án xe Micromouse thông minh tốc độ cao: Tự động khảo sát sa bàn, phát hiện vật cản bằng cảm biến Laser ToF, quy hoạch quỹ đạo tối ưu bằng thuật toán A* và điều khiển bứt tốc bám đường với bộ điều khiển PID vận tốc điều khiển động cơ bằng PWM.

---

##  1. Tổng Quan Dự Án

Khác với các dòng Micromouse truyền thống chỉ chạy thuật toán quay phải/trái hay Floodfill cơ bản, hệ thống hướng tới mô hình thi đấu 2 vòng độc lập:

1. **Vòng 1 (Khảo sát / Exploration & Mapping):**
   - Xe tự động thám hiểm theo tuyến đường định trước hoặc điều khiển từ xa qua ESP32-S3.
   - 3 cảm biến Laser ToF (VL53L1X tầm xa phía trước, 2x VL53L0X hai bên sườn) quét liên tục để nhận diện các ô trống và phát hiện vị trí vật cản ngẫu nhiên.
   - Tọa độ vật cản được cập nhật trực tiếp vào ma trận lưới bản đồ kích thước **60 x 40 ô** (độ phân giải 3cm/ô tương ứng sa bàn $1800\text{ mm} \times 1200\text{ mm}$).
2. **Vòng 2 (Chạy tốc độ cao / Speed Run):**
   - Dựa trên bản đồ đã thám hiểm, thuật toán **A* (A-Star)** tính toán chuỗi điểm mốc đường đi ngắn nhất, tránh toàn bộ vật cản.
   - Xe chạy bứt tốc, ứng dụng kỹ thuật **ôm cua uốn lượn liên tục (Continuous Dynamic Arc)**: Khi tới góc rẽ, xe uốn cong mượt mà theo cung tròn mà **không dừng lại xoay góc tại chỗ**, bảo toàn động năng và hoàn thành đường đua trong thời gian ngắn nhất.

---

## 2. Phần Cứng

| Linh Kiện | Số Lượng | Mô Tả & Thông Số Kỹ Thuật | Vai Trò Trong Xe |
|---|:---:|---|---|
| **MCU STM32F401RCT6** | 1 | ARM Cortex-M4 có FPU, 84 MHz, 256KB Flash, 64KB SRAM (LQFP64) | Xử lý trung tâm: Odometry, đọc cảm biến, bù trôi GyroZ & ZUPT, thuật toán A*, PID động cơ |
| **ESP32-S3 Mini** | 1 | Vi điều khiển kép Xtensa LX7, Wi-Fi 2.4GHz & BLE 5.0 | Mạch đế truyền thông: Cầu nối UART ↔ Web Dashboard (USB Serial & Wi-Fi WebSocket) |
| **Động cơ N20 + Encoder** | 2 | Động cơ giảm tốc kim loại N20 (99:1), Encoder từ 28 xung/vòng motor | Dẫn động 2 bánh vi sai độc lập, phản hồi xung góc quay chính xác |
| **Driver Động Cơ** | 1 | Mạch cầu H (tận dụng từ servo MG90S) | Nhận xung điều khiển PWM 50Hz từ STM32 để kéo 2 động cơ N20 |
| **Cảm Biến ToF VL53L1X** | 1 | Laser ToF tầm xa (đo cự ly tới 4000mm), giao tiếp I2C1 | Đặt chính giữa đầu xe để đo khoảng cách vật cản phía trước |
| **Cảm Biến ToF VL53L0X** | 2 | Laser ToF tầm gần (đo cự ly tới 2000mm), giao tiếp I2C1 | Đặt hai bên sườn (Trái góc +45°, Phải góc -45°) đo khoảng cách vách |
| **Cảm Biến IMU MPU6500** | 1 | Cảm biến Gyro & Accel 6 trục, giao tiếp bus I2C2 riêng biệt | Đo vận tốc góc trục Z ($Gyro_Z$) nhằm bù góc trượt và định hướng thân xe |
| **Pin LiPo 2S** | 1 | 7.4V danh định (8.4V khi sạc đầy) | Cung cấp nguồn năng lượng chính cho toàn bộ xe |
| **Mạch Hạ Áp Buck 6V** | 1 | Mạch nguồn hạ áp DC-DC từ Pin 2S xuống 6.0V ổn định | Cấp nguồn công suất riêng cho động cơ N20 và làm đầu vào cho LDO 3.3V |
| **Mạch LDO AMS1117-3.3V** | 1 | Hạ áp tuyến tính Low-Dropout từ 6.0V xuống 3.3V sạch (1A) | Cấp nguồn Logic cho STM32, ESP32, 3x ToF, MPU6500, 2x Encoders |
| **Ngoại vi UI** | 3 LED, 2 Nút | 3 LED báo trạng thái, 2 nút nhấn chọn chế độ chạy | Thao tác kích hoạt, và báo lỗi. |

---

##  3. Sơ Đồ Nguyên Lý Mạch Điện

Chi tiết kết nối chân vi điều khiển và sơ đồ nguyên lý phần cứng toàn xe:

**Sơ đồ nguyên lý:** [micromouse_schematic.png](micromouse_schematic.png)


---

##  4. Kiến Trúc Điều Khiển & Thuật Toán Nâng Cao

### 4.1. Bộ Điều Khiển PID Vận Tốc Điều Khiển Động Cơ Bằng Độ Rộng Xung PWM ($\mu s$)
* **Độ phân giải $1\mu s$:** Timer TIM1 và TIM11 được cấu hình xung nhịp đếm $1\text{ MHz}$ (`ARR = 19999`, tần số $50\text{ Hz}$).Vòng lặp kín PID tính toán trực tiếp độ rộng xung theo micro-giây:
  $$\text{Pulse}(\mu s) = \text{Offset} \pm \text{Deadzone} \pm \Delta\mu s_{\text{PID}}$$
* **Derivative on Measurement:** Áp dụng vi phân trên biến đo vận tốc thực tế ($-\frac{\Delta v}{\Delta t}$) triệt tiêu hoàn toàn xung giật vi phân (Derivative Kick) khi đổi setpoint đột ngột.
* **Tự động bù Deadzone tĩnh:** Vượt qua ma sát cơ khí khởi động tức thời mà không bị trễ nấc.

### 4.2. Thuật Toán Ôm Cua Bảo Toàn Vận Tốc (Continuous Arc Cornering)
* **Không dừng xe xoay tại chỗ:** Khi chuyển tiếp giữa các điểm mốc (Waypoints), xe uốn lượn liên tục theo cung tròn mượt mà.
* **Giữ nguyên tốc độ bánh ngoài:** nhằm giữ vận tốc ổn định.
* **Bộ lọc vi sai Slew-Rate Limiter ($14\text{ mm/s}$ mỗi $10\text{ ms}$):** Giới hạn tốc độ biến thiên lực kéo, bảo toàn quán tính thân xe và tránh hiện tượng khựng giật bánh xe.

### 4.3. Thuật Toán Tìm Đường A* (A-Star) Trên STM32
* Không gian tìm kiếm: Ma trận lưới $60 \times 40$ ô (mỗi ô $30\text{ mm}$, tổng 2400 ô nhớ trong RAM).
* Quản lý vật cản tĩnh cố định và tự động phát hiện, xác nhận vật cản ngẫu nhiên từ dữ liệu ToF thời gian thực.
* Tự động làm mịn đường đi và trích xuất chuỗi điểm mốc điều hướng tối ưu.

### 4.4. Bù Trôi Gyro Z & Khóa Đứng Yên ZUPT (Gyro Z + Wheel Odometry)
* Dùng vận tốc góc từ Gyro Z MPU6500 đã bù trôi kết hợp khóa đứng yên ZUPT khi hai bánh dừng (dự phòng Encoder khi không có Gyro), loại bỏ hiện tượng trôi góc khi đứng yên và sai số lượng tử hóa khi quay chậm.

---

## 📡 5. Cấu Trúc Khung Tin Giao Tiếp UART (921600 Baud)

### 5.1. Khung Điều Khiển Từ ESP32 Xuống STM32 (8 Byte, Tần số 50 Hz)
| Byte Offset | Giá Trị / Kiểu | Tên Trường | Ý Nghĩa |
|:---:|:---:|---|---|
| `0` | `0xAA` | Header 1 | Byte nhận dạng đồng bộ 1 |
| `1` | `0xBB` | Header 2 | Byte nhận dạng đồng bộ 2 |
| `2` | `uint8_t` | Command | Mã lệnh (`0x00`: Manual, `0x03`: Rotate, `0x04`: Reset Odom, `0x06`: NavToPoint, `0x08`: SetObs) |
| `3..4` | `int16_t` (Big-Endian) | Data 1 | Tham số 1 (Tốc độ bánh phải / Góc xoay / Tọa độ X / Slot Top) |
| `5..6` | `int16_t` (Big-Endian) | Data 2 | Tham số 2 (Tốc độ bánh trái / Tọa độ Y / Slot Bottom) |
| `7` | `uint8_t` | `checksum` | Checksum cộng dồn 8-bit modulo 256 tính trên `Byte 0..6`: `(Byte[0] + ... + Byte[6]) & 0xFF` |

### 5.2. Khung Telemetry Từ STM32 Lên ESP32 / Web (32 Byte, Tần số 40 Hz)
| Byte Offset | Kiểu Dữ Liệu | Trường Dữ Liệu | Ý Nghĩa & Đơn Vị |
|:---:|:---:|---|---|
| `0..1` | `0xAA 0xBB` | Header Frame | Byte nhận dạng đầu khung |
| `2..3` | `int16_t` | `data1` | Tọa độ $X$ của xe (mm) |
| `4..5` | `int16_t` | `data2` | Tọa độ $Y$ của xe (mm) |
| `6..7` | `int16_t` | `data3` | Góc hướng $Heading$ của xe (độ) |
| `8..9` | `int16_t` | `data4` | Vận tốc góc $Gyro_Z$ (°/s) |
| `10..11` | `int16_t` | `data5` | Tọa độ $X$ điểm chạm ToF Trái |
| `12..13` | `int16_t` | `data6` | Tọa độ $Y$ điểm chạm ToF Trái |
| `14..15` | `int16_t` | `data7` | Tọa độ $X$ điểm chạm ToF Giữa |
| `16..17` | `int16_t` | `data8` | Tọa độ $Y$ điểm chạm ToF Giữa |
| `18..19` | `int16_t` | `data9` | Tọa độ $X$ điểm chạm ToF Phải (mm) |
| `20..21` | `int16_t` | `data10` | Tọa độ $Y$ điểm chạm ToF Phải (mm) |
| `22..23` | `int16_t` | `data11` | Khoảng cách cảm biến ToF Trái (mm) |
| `24..25` | `int16_t` | `data12` | Khoảng cách cảm biến ToF Giữa (mm) |
| `26..27` | `int16_t` | `data13` | Khoảng cách cảm biến ToF Phải (mm) |
| `28..29` | `int16_t` | `data14` | Tải CPU STM32 |
| `30..31` | `int16_t` | `data15` | Trạng thái chuyển động: `(activeCommand << 8) | commandStatus` |


---

## 📂 6. Cấu Trúc Mã Nguồn Firmware

```text
micromouse/
├── Inc/
│   ├── astar.h               # Thuật toán tìm đường A* và quản lý vòng chạy 1 & 2
│   ├── button.h              # Xử lý nút bấm (click, double click, hold)
│   ├── config.h              # Cấu hình toàn bộ thông số: PID (us), Odometry, GyroZ/ZUPT, ngưỡng cua
│   ├── encoder.h             # Đọc xung EXTI từ encoder 2 bánh
│   ├── i2c_bus.h             # Giao tiếp I2C1 (ToF) & I2C2 (MPU6500)
│   ├── gyro_zupt.h           # Bù trôi Gyro Z & khóa đứng yên ZUPT cho góc hướng Odometry
│   ├── led.h                 # Điều khiển LED đa chế độ (chớp, sáng liên tục, báo lỗi)
│   ├── map_grid.h            # Ma trận lưới bản đồ 60x40 ô & hàm cập nhật tia ToF
│   ├── motor_pwm.h           # Điều khiển PWM TIM1 & TIM11 trực tiếp theo micro-giây (us)
│   ├── mpu6500.h             # Driver đọc cảm biến IMU MPU6500
│   ├── navigation.h          # Tính toán tọa độ hình học tia quét cảm biến ToF
│   ├── odometry.h            # Tính toán vị trí xe X, Y, Heading và vận tốc tuyến tính
│   ├── pid_controller.h      # Bộ điều khiển PID vận tốc bánh xe & bám hướng ôm cua
│   ├── tof_sensors.h         # Quản lý 3 cảm biến ToF (VL53L1X + 2x VL53L0X)
│   ├── tof_xshut.h           # Điều khiển chân XSHUT định địa chỉ I2C cảm biến ToF
│   └── uart_protocol.h       # Khung truyền thông UART DMA 921600 baud
├── Src/
│   ├── astar.c
│   ├── config.c
│   ├── encoder.c
│   ├── gyro_zupt.c
│   ├── main.c               # Vòng lặp chính, lập lịch task cooperative 1000Hz/100Hz/40Hz
│   ├── map_grid.c
│   ├── motor_pwm.c
│   ├── mpu6500.c
│   ├── navigation.c
│   ├── odometry.c
│   ├── pid_controller.c
│   ├── tof_sensors.c
│   └── uart_protocol.c
├── STM32F401RCTX_FLASH.ld    # Linker script cấu hình vùng nhớ và Stack an toàn 4KB
└── README.md
```

---

## 🛠️ 7. Hướng Dẫn Biên Dịch & Nạp Code

* **Trình biên dịch:** GNU Arm Embedded Toolchain (`arm-none-eabi-gcc 14.3+`)
* **Môi trường:** STM32CubeIDE hoặc Visual Studio Code kèm `make`.
* **Biên dịch qua dòng lệnh:**
  ```powershell
  cd Debug
  make all -j8
  arm-none-eabi-objcopy -O ihex micromouse.elf micromouse.hex
  arm-none-eabi-objcopy -O binary micromouse.elf micromouse.bin
  ```
* **Nạp Firmware:** Dùng ST-Link qua cổng SWD trên mạch hoặc phần mềm **STM32CubeProgrammer**.
