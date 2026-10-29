# Giao diện điều khiển cánh tay robot 6DOF

Điều khiển cánh tay bằng **thanh trượt** hoặc **joystick** (cần ảo trên màn hình cảm ứng, hoặc tay cầm Xbox/PS thật) từ trình duyệt trên **máy tính hoặc điện thoại**.

```
Trình duyệt (PC / điện thoại) --WebSocket--> server.py --UART 115200--> STM32F411 (RobotArm_6DOF) --PWM--> 6 servo
```

## 1. Nạp firmware

Mở `RobotArm_6DOF/MDK-ARM/RobotArm_6DOF.uvprojx` bằng Keil, build và nạp như bình thường.
Firmware nhận lệnh trên **cả hai UART**:

| UART | Chân | Dùng cho |
|------|------|----------|
| USART2 (`LAPTOP`) | PA2 (TX), PA3 (RX) | Laptop qua mạch USB-UART (CP2102/CH340…) |
| USART1 (`BLE`)    | PA9 (TX), PA10 (RX) | Module BLE/Bluetooth nối với điện thoại |

Nối chéo TX↔RX và **chung GND**. Nguồn servo cấp riêng (không lấy từ chân 3V3/5V của board), chung GND với STM32.

## 2. Chạy giao diện web

```bash
cd control_ui
pip install -r requirements.txt

python server.py --list                 # xem tên cổng serial
python server.py --serial COM5          # Windows
python server.py --serial /dev/ttyUSB0  # Linux / Raspberry Pi
python server.py --sim                  # thử giao diện khi chưa có phần cứng
```

Server in ra hai địa chỉ:

- **Máy tính:** `http://localhost:8080`
- **Điện thoại** (cùng Wi-Fi với máy chạy server): `http://<IP-máy-tính>:8080`
  (Windows hỏi tường lửa thì chọn *Allow* cho mạng riêng tư.)

Tùy chọn: `--baud`, `--http-port`, `--host 127.0.0.1` (chỉ cho máy này), `--token matkhau` (khi đó mở trang bằng `/?token=matkhau`).

> Server không có xác thực mặc định và có thể điều khiển phần cứng thật. Chỉ chạy trên mạng tin cậy, hoặc dùng `--token`.

## 3. Cách dùng

**Tab "Thanh trượt":** mỗi khớp một thanh 0–180°.

**Tab "Joystick":** gạt cần để quay khớp theo vận tốc, thả tay thì khớp đứng yên.
- Mỗi cần có 2 ô chọn để gán khớp cho trục X/Y và tick "đảo" nếu chiều quay ngược. Mặc định: cần trái = Đế + Vai, cần phải = Cổ tay gập + Khuỷu.
- Nút `−` / `+` giữ để quay Cổ tay xoay và Kẹp.
- Tay cầm vật lý (Gamepad API): cần trái/phải = 2 joystick, LB/RB = cổ tay xoay, LT/RT = kẹp, A = về vị trí gốc.
  Trình duyệt chỉ cho dùng tay cầm trên trang `localhost` hoặc HTTPS, nên cắm tay cầm vào **máy tính** và mở `http://localhost:8080` (trên điện thoại qua IP LAN thì dùng cần ảo).

**Chung:** thanh *Tốc độ servo* (°/s) giới hạn tốc độ tối đa, firmware làm mượt chuyển động nên kéo thanh trượt nhanh cũng không bị giật. Nút *Về vị trí gốc* đưa tất cả khớp về 90°.

Nhiều thiết bị có thể mở trang cùng lúc, vị trí các khớp được đồng bộ giữa chúng.

## 4. Giao thức UART (dòng ASCII, kết thúc `\n`)

| Lệnh | Ví dụ | Ý nghĩa |
|------|-------|---------|
| `a1,a2,a3,a4,a5,a6` | `90,45.5,120,90,90,30` | Góc đích 6 servo (độ). Giống giao thức của project `RobotArm` (F103). |
| `Sn:góc` | `S3:135` | Đặt riêng servo n (1–6) |
| `Vtốc_độ` | `V90` | Tốc độ tối đa, 5–720 °/s |

Nhờ vậy cũng có thể điều khiển từ điện thoại bằng một app "BLE/Bluetooth Serial Terminal" bất kỳ, gõ lệnh trực tiếp.

## 5. Tùy chỉnh cho cánh tay của bạn

- **Giới hạn góc từng khớp** (tránh va chạm cơ khí): `servo_min[]`/`servo_max[]` trong `RobotArm_6DOF/Core/Src/main.c` và `JOINTS` (min/max) ở đầu script trong `static/index.html`.
- **Xung servo:** `SERVO_MIN_US`/`SERVO_MAX_US` trong `main.c` (mặc định 1000–2000 µs như project F103; nhiều servo MG996R/MG90S dùng 500–2500 µs).
- **Servo quay ngược:** tick "đảo" ở tab Joystick, hoặc đổi chiều trong firmware.
- **Mất kết nối:** các lệnh là góc đích tuyệt đối, nên nếu trình duyệt/server tắt đột ngột, cánh tay dừng lại ở vị trí đích cuối cùng chứ không chạy tiếp.
