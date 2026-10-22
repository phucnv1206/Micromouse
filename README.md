# 🐭 Micromouse Fast-Runner (STM32F401RCT6)

> Dự án xe dò đường / chạy map thông minh: Cho phép chạy thủ công trước trong sa bàn để thu thập dữ liệu và lập bản đồ, sau đó xe tự động tối ưu đường đi để chạy lại với tốc độ nhanh nhất có thể (**Speed Run**).

---

## 📖 1. Tổng Quan Dự Án

Khác với các dòng Micromouse truyền thống phải tự mò mẫm mê cung hoàn toàn từ đầu theo luật bàn tay phải hay Floodfill thời gian thực, dự án này hướng đến mô hình:
1. **Pha 1 (Khảo sát / Mapping):** Xe được điều khiển thủ công (Manual Run) thông qua kết nối không dây của ESP32-S3 để đi qua các cung đường trong map, vừa chạy vừa ghi nhận tọa độ Odometry và khoảng cách vật cản.
2. **Pha 2 (Tối ưu & Tăng tốc / Speed Run):** Dựa trên quỹ đạo đã thu thập, vi điều khiển tính toán quỹ đạo tối ưu, bo cua mượt và điều khiển động cơ bứt tốc tối đa để hoàn thành vòng chạy trong thời gian ngắn nhất.

> [!NOTE]
> Tài liệu này tập trung vào **kiến trúc phần cứng, phân phối nguồn điện và cách thức điều khiển các module ngoại vi**. Các thuật toán tìm đường và tối ưu tốc độ sẽ được phát triển tiếp ở các pha sau.

---

## 🛠️ 2. Danh Mục Phần Cứng (BOM)

| Linh Kiện | Số Lượng | Mô Tả & Thông Số Kỹ Thuật | Vai Trò Trong Xe |
|---|:---:|---|---|
| **MCU STM32F401RCT6** | 1 | ARM Cortex-M4, xung nhịp 84 MHz, 256KB Flash, 64KB SRAM (LQFP64) | Xử lý trung tâm: Odometry, đọc cảm biến, PID động cơ, lập lịch task |
| **ESP32-S3 Mini** | 1 | Vi điều khiển kép Xtensa LX7, tích hợp Wi-Fi 2.4GHz & BLE 5.0 | Mạch đế giao tiếp: Cầu nối UART ↔ Máy tính (Debug/Telemetry), tương lai làm trạm phát Wi-Fi nhận lệnh điều khiển xe thủ công |
| **Động cơ N20 + Encoder** | 2 | Động cơ giảm tốc N20 micro kim loại, tỉ số truyền 99:1, tích hợp Encoder quang/từ 28 xung/vòng motor | Dẫn động 2 bánh vi sai độc lập, phản hồi xung góc quay chính xác |
| **Driver Động Cơ** | 1 | Module cầu H (lấy trong servo mg90) | Nhận xung điều khiển PWM từ STM32 để kéo 2 động cơ N20 |
| **Cảm Biến ToF VL53L1X** | 1 | Cảm biến khoảng cách Laser ToF tầm xa (đến 4m), giao tiếp I2C1 | Đặt ở **chính giữa đầu xe** để đo cự ly vật cản phía trước |
| **Cảm Biến ToF VL53L0X** | 2 | Cảm biến Laser ToF tầm gần (đến 2m), giao tiếp I2C1 | Đặt ở **hai bên sườn (Trái/Phải)** để đo khoảng cách vách |
| **Cảm Biến IMU MPU6050** | 1 | Cảm biến con quay hồi chuyển & gia tốc 6 trục (dùng I2C2 riêng) | Đo vận tốc góc trục Z ($Gyro_Z$) nhằm bù góc trượt và định hướng thân xe |
| **Pin LiPo 2S** | 1 | 7.4V danh định (8.4V sạc đầy) | Cung cấp nguồn năng lượng chính cho toàn bộ xe |
| **Mạch Hạ Áp Buck 6V** | 1 | Mạch nguồn hạ áp DC-DC từ Pin 2S (7.4V - 8.4V) xuống ổn định 6.0V | Cấp nguồn công suất riêng cho động cơ N20 qua Driver và làm đầu vào cho LDO 3.3V |
| **Mạch LDO AMS1117-3.3V** | 1 | Mạch hạ áp tuyến tính Low-Dropout hạ từ **6.0V xuống 3.3V** (dòng tải 800mA - 1A, kèm tụ lọc input/output) | Cấp nguồn Logic sạch cho STM32, ESP32, 3x ToF, MPU6050, 2x Encoders |
| **Ngoại vi UI** | 3 LED, 2 Nút | 3 LED báo trạng thái, 2 nút nhấn cấu hình chế độ chạy | Thao tác chọn chế độ, báo lỗi, căn chỉnh xe tại vạch xuất phát |
| **Tụ lọc & Đầu nối** | - | Tụ điện phân (100uF - 220uF), tụ gốm (100nF), Jack XT30, XH2.54 | Chống sụt áp khi đề-pa động cơ, lọc nhiễu đường nguồn 3.3V và 6V |

---

## ⚡ 3. Sơ Đồ Phân Phối Nguồn Điện (Power Tree)

Hệ thống sử dụng mô hình hạ áp **nối tầng (Cascade)**: Pin 2S trước tiên qua Buck DC-DC hiệu suất cao hạ xuống **6.0V** cấp cho động cơ, sau đó từ đường 6.0V này mới qua LDO **AMS1117-3.3V** để tạo ra điện áp logic 3.3V sạch, phẳng và ít tỏa nhiệt hơn:

```
                      +-----------------------------+
                      |     PIN LIPO 2S (7.4V)      |
                      +--------------+--------------+
                                     |
                                     v [7.4V - 8.4V]
                      +-----------------------------+
                      |      MẠCH BUCK DC-DC 6V     |
                      +--------------+--------------+
                                     |
               +---------------------+---------------------+
               | [Đường Nguồn Động Lực 6.0V]               | [Đường Nguồn Vào LDO]
               v                                           v
+-----------------------------+             +-----------------------------+
| Driver Motor (Cầu H Servo)  |             |      LDO AMS1117-3.3V       |
| + 2x Động Cơ N20            |             +--------------+--------------+
+--------------+--------------+                            |
               |                                           v [Nguồn Logic Sạch 3.3V]
               |                            +-----------------------------+
               |                            | • STM32F401RCT6             |
               |                            | • ESP32-S3 Mini             |
               |                            | • 1x VL53L1X + 2x VL53L0X   |
               |                            | • MPU6050 (IMU)             |
               |                            | • 2x Encoders               |
               |                            +--------------+--------------+
               |                                           |
               +---------------------+---------------------+
                                     |
                                     v
                        [ COMMON GROUND - MASS CHUNG ]
```

> [!TIP]
> **Ưu điểm của thiết kế nối tầng (7.4V $\rightarrow$ 6.0V $\rightarrow$ 3.3V):**
> - Độ sụt áp qua LDO AMS1117 chỉ còn $\Delta V = 6.0V - 3.3V = 2.7V$ (thay vì $8.4V - 3.3V = 5.1V$ nếu hạ trực tiếp từ pin sạc đầy). Nhờ đó, LDO hoạt động mát hơn rất nhiều, giảm nguy cơ quá nhiệt gây ngắt nguồn bảo vệ trong lúc xe đang chạy.
> - Dropout voltage của AMS1117 khoảng $1.1V - 1.3V$, do đó đầu vào $6.0V$ là khoảng điện áp lý tưởng để đầu ra $3.3V$ luôn ổn định tuyệt đối ngay cả khi tải động cơ thay đổi.

> [!WARNING]
> **Bắt buộc nối chung toàn bộ Mass (GND)** giữa nguồn 6V, nguồn 3.3V, mạch STM32, Driver và ESP32 để đảm bảo các mức điện áp logic điều khiển (PWM, EXTI, UART, I2C) không bị trôi điểm mốc.

---

## 🔌 4. Bảng Đấu Nối Chân Vi Điều Khiển (STM32F401 Pinout)

Hệ thống được thiết kế tối ưu với **2 Bus I2C tách biệt** nhằm giúp việc đọc cảm biến góc MPU6050 ở tần số cao không bị xung đột với 3 cảm biến ToF:

| Khối Chức Năng | Chân STM32 | Chế Độ Ngoại Vi | Nối Đến Linh Kiện |
|---|---|---|---|
| **Motor Phải** | `PA8` | TIM1_CH1 (PWM 50Hz) | Chân điều khiển tốc độ/chiều bánh phải |
| **Motor Trái** | `PB9` | TIM11_CH1 (PWM 50Hz) | Chân điều khiển tốc độ/chiều bánh trái |
| **Encoder Phải** | `PA12` / `PA11` | EXTI12 / EXTI11 | Kênh Pha A & Pha B Encoder bánh phải |
| **Encoder Trái** | `PB4` / `PB5` | EXTI4 / EXTI5 | Kênh Pha A & Pha B Encoder bánh trái |
| **Bus I2C1 (ToF)** | `PB6` / `PB7` | I2C1 SCL / SDA (400kHz) + DMA | Nối chung đến chân SCL, SDA của cả 3 ToF |
| **XSHUT ToF Trái** | `PB15` | GPIO Output | Chân XSHUT của cảm biến VL53L0X Trái |
| **XSHUT ToF Giữa** | `PB14` | GPIO Output | Chân XSHUT của cảm biến VL53L1X Giữa |
| **XSHUT ToF Phải** | `PB13` | GPIO Output | Chân XSHUT của cảm biến VL53L0X Phải |
| **Bus I2C2 (MPU)** | `PB10` / `PB3` | I2C2 SCL (AF4) / SDA (AF9) | Nối riêng đến SCL, SDA của MPU6050 |
| **UART Telemetry** | `PA2` / `PA3` | USART2 TX / RX (921600 baud) | Nối chéo (TX-RX, RX-TX) sang **ESP32-S3 Mini** |
| **Nút Nhấn** | `PA0` / `PA1` | GPIO Input (Pull-up nội) | 2 nút bấm thao tác nối xuống GND |
| **LED Trạng Thái** | `PC13`..`PC15` | GPIO Output | 3 LED nối tiếp qua trở 330Ω xuống GND |

---

## 📡 5. Vai Trò Của ESP32-S3 Mini (Bridge & Remote Control)

Module **ESP32-S3 Mini** đóng vai trò là "mạch đế" thông minh kết nối không dây cho STM32:

1. **Giai đoạn Hiện Tại (Debug & Telemetry Bridge):**
   - STM32 gửi gói dữ liệu Telemetry (16 byte ở tần số 40 Hz: $x, y, heading$, ToF Trái/Giữa/Phải, CPU load) qua cổng `USART2` tốc độ **921600 baud**.
   - ESP32 nhận chuỗi dữ liệu này và đẩy thẳng về máy tính qua cổng USB CDC hoặc UDP/WebSocket để người dùng theo dõi biểu đồ thời gian thực.
2. **Giai đoạn Tương Lai (Manual Drive & Mapping):**
   - ESP32 phát mạng Wi-Fi Access Point (AP) hoặc nhận tín hiệu từ tay cầm BLE.
   - Người điều khiển lái xe chạy một vòng sa bàn thủ công. Lệnh điều khiển được ESP32 đóng gói gửi tức thời sang STM32 qua UART để xe chạy theo ý muốn, đồng thời thu thập toàn bộ bản đồ map.

---

## 🧠 6. Kiến Trúc Firmware STM32 (Non-Blocking & Real-Time)

Firmware được tổ chức dạng **Cooperative Scheduler** nhiều chu kỳ nhịp nhàng, tối ưu hóa vòng quét CPU:

```
[TIM3 Interrupt (1000 Hz)]
         │
         ├──> Odometry_Update()   : Tích phân vi sai từ xung Encoder
         └──> Mpu6050_Update()    : Đọc Gyro Z qua I2C2 + Lọc IIR Low-Pass

[Task Điều Khiển (100 Hz)]
         └──> Đọc lệnh điều khiển UART / Cập nhật độ rộng xung PWM động cơ

[Task Telemetry (40 Hz)]
         ├──> Đọc khoảng cách 3 cảm biến ToF (Chạy ngầm bằng DMA1)
         ├──> Yield Callback: Cho phép Odometry chen ngang trong lúc ToF DMA đang bận
         └──> Gửi gói tin Telemetry 16 byte về ESP32 qua UART
```

### Điểm nổi bật trong kiến trúc:
* **Tách rời 2 Bus I2C:** Bus `I2C2` chạy riêng cho MPU6050 ở tần số 1000 Hz, trong khi `I2C1` chạy cho 3 ToF ở 40 Hz.
* **Cơ chế Yield Callback chống nghẽn:** Khi ToF đang trao đổi dữ liệu với DMA trên `I2C1`, CPU không đứng chờ lãng phí mà liên tục kiểm tra cờ nhịp `TIM3`. Nếu đến hạn 1ms, MCU thực hiện ngay `Odometry_Update()` và đọc `MPU6050` qua `I2C2`, giúp việc tính toán vị trí xe không bao giờ bị trễ dù cảm biến khoảng cách đang đo.
* **Bộ lọc thông thấp số (IIR Low-Pass Filter):** Cả dữ liệu đo khoảng cách ToF và tốc độ góc $Gyro_Z$ đều được lọc mượt, loại bỏ các gai nhiễu cơ khí.

---

## 💻 7. Quy Trình Làm Việc: Kết Hợp VS Code & STM32CubeIDE

Mô hình phát triển tối ưu cho dự án:
* **STM32CubeIDE:** Dùng để cấu hình chân/ngoại vi (`micromouse.ioc`), **Biên dịch (Build)** và **Nạp/Debug (Flash)** xuống xe qua ST-Link.
* **Visual Studio Code:** Dùng làm **trình soạn thảo code chính (Editor)** nhờ giao diện mượt, phím tắt tiện lợi, tìm kiếm nhanh và hỗ trợ AI.

---

## ⚡ 8. Hướng Dẫn Cấu Hình VS Code Để Soạn Thảo (Không Báo Lỗi Đỏ)

Mục tiêu là cấu hình trình phân tích cú pháp (IntelliSense) của VS Code nhận diện đúng trình biên dịch **ARM GCC (`arm-none-eabi-gcc`)** và các thư viện HAL STM32, giúp gợi ý hàm (Autocomplete), nhảy tới định nghĩa (`F12`), và **loại bỏ hoàn toàn các gạch chân báo lỗi đỏ** (`#include`, `uint32_t`, `HAL_OK`...).

### 8.1. Cài đặt Extension cần thiết:
Chỉ cần cài đặt duy nhất 1 extension trên VS Code:
* **C/C++** (của Microsoft, ID: `ms-vscode.cpptools`).

### 8.2. Mở dự án và cấu hình:
1. Mở VS Code → chọn **File** → **Open Folder...** → chọn thư mục `micromouse`.
2. Dự án đã được tạo sẵn file cấu hình [`.vscode/c_cpp_properties.json`](.vscode/c_cpp_properties.json) chứa toàn bộ các đường dẫn Include và cờ chip STM32F401:
   * Đường dẫn thư viện: `Inc`, `Drivers/STM32F4xx_HAL_Driver`, `Drivers/CMSIS`, `Drivers/VL53L0X`, `Drivers/VL53L1X`.
   * Cờ vi xử lý: Cortex-M4 có FPU (`-mcpu=cortex-m4`, `-mfpu=fpv4-sp-d16`, `-mfloat-abi=hard`).
   * Defines: `USE_HAL_DRIVER`, `STM32F401xC`, `DEBUG`.

### 8.3. Nếu VS Code vẫn báo lỗi gạch đỏ ở các dòng `#include`:
Nguyên nhân là do đường dẫn compiler GCC trên máy bạn khác với đường dẫn mặc định trong file config. Bạn chỉ cần cập nhật lại:

1. Mở **PowerShell** và chạy lệnh sau để tìm vị trí `arm-none-eabi-gcc.exe` đã được cài sẵn cùng STM32CubeIDE trên máy:
   ```powershell
   Get-ChildItem "C:\ST" -Filter "arm-none-eabi-gcc.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -ExpandProperty FullName
   ```
2. Mở file [`.vscode/c_cpp_properties.json`](.vscode/c_cpp_properties.json) trong VS Code.
3. Dán đường dẫn vừa tìm được vào mục `"compilerPath"`. *(Lưu ý: dùng dấu gạch chéo xuôi `/` thay vì gạch chéo ngược `\`)*.
4. Lưu file (`Ctrl + S`). VS Code sẽ tự động phân tích lại toàn bộ project và sạch bóng các lỗi gạch chân đỏ.

---

## 🗺️ 9. Kế Hoạch Phát Triển Tiếp Theo
- [ ] Hoàn thiện code nhận lệnh và truyền nhận dữ liệu trên ESP32-S3 Mini.
- [ ] Xây dựng thuật toán lọc phối hợp Odometry bánh xe và MPU6050 (Complementary / Kalman Filter).
- [ ] Lưu trữ và tái hiện quỹ đạo chạy (Mapping & Replay Trajectory).
- [ ] Thuật toán tối ưu góc cua (Curve optimization) để đạt tốc độ tối đa trong pha Fast-Run.

