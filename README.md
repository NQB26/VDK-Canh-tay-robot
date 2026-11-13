<<<<<<< Updated upstream
# VDK-Canh-tay-robot
=======
# VDK-Canh-tay-robot

## Tổng quan dự án

Đề tài: **Thiết kế và điều khiển mô hình cánh tay robot công nghiệp thu nhỏ**.

Dự án hướng đến cánh tay robot sử dụng 6 servo, có khả năng gắp các vật nhẹ, điều khiển thủ công từ máy tính hoặc điện thoại, sau đó mở rộng sang chạy chuỗi thao tác tự động và ROS 2.

Mục tiêu tải trọng đặt ra là **100–300 g**. Đây là **mục tiêu thiết kế**, chưa phải tải trọng đã đo kiểm hoặc được bảo đảm bởi mô hình hoàn thiện.

### Mục tiêu và trạng thái hiện tại

| Mục tiêu | Trạng thái | Minh chứng / phần còn thiếu |
| --- | --- | --- |
| Thiết kế mô hình cánh tay robot công nghiệp thu nhỏ, tải nhẹ | **Đang thực hiện** | Đã có bộ chi tiết STL, lựa chọn 3 MG996R và 3 MG90S, xây dựng phần điều khiển STM32. Chưa có kết quả đo tải gắp 100–300 g, độ võng hoặc độ bền khi vận hành liên tục. |
| Điều khiển bằng joystick hoặc thanh trượt trên máy tính / điện thoại | **Hoàn thành một phần** | Đã kết nối HC-06 với ứng dụng Bluetooth Serial trên điện thoại; đã thử truyền lệnh và điều khiển các khớp bằng lệnh văn bản. Chưa làm giao diện joystick/slider; chưa hoàn thiện điều khiển qua máy tính. |
| Cánh tay tự thực hiện một tổ hợp thao tác | **Chưa triển khai** | Cần xây dựng chức năng lưu tư thế, phát lại danh sách tư thế, điều khiển thời gian và cơ chế dừng an toàn. |
| Tích hợp ROS 2 để điều khiển robot (nâng cao) | **Có mã thử nghiệm, chưa tích hợp** | Thư mục `RobotArm/scr/` chứa các bản nháp Python dùng `rclpy` cho động học nghịch và truyền lệnh. Chưa chứng minh được chuỗi ROS 2 → máy tính → STM32F411 → servo hoạt động hoàn chỉnh. |

### Phần đã thực hiện

**Phần cơ khí và phần cứng**

- Đã xây dựng các tệp STL cho đế, các đoạn tay, khớp xoay và cơ cấu kẹp trong `3D PARTS/`.
- Đã chọn 6 servo gồm **3 × MG996R** và **3 × MG90S**. Việc gán từng loại servo vào J1–J6 cần được xác nhận theo bản lắp thực tế.
- Đã lựa chọn STM32F411CEU6 làm bộ điều khiển và cấu hình 6 kênh PWM trên TIM3/TIM4.
- Đã nghiên cứu phương án in PETG trên Bambu Lab H2D, ưu tiên chi tiết nhẹ. Chưa ghi nhận đầy đủ kết quả thử tải cơ khí của mô hình.

**Phần lập trình và giao tiếp**

- Đã tạo cấu hình STM32CubeMX (`.ioc`) và firmware STM32 HAL cho PWM điều khiển 6 servo.
- Đã cấu hình UART1 để kết nối **HC-06 (Bluetooth Classic SPP, 9600 baud)**; người thực hiện đã xác nhận điện thoại kết nối và nhận được dữ liệu.
- Đã xây dựng mã xử lý lệnh `PING`, `HELP`, `STATUS`, `J1 90`, `J1 +1`, `J1 -1`, `STOP`, `OFF 1` và `OFF ALL`.
- Đã kiểm tra chuyển động thực tế của servo MG996R và MG90S; hiện **đang hiệu chỉnh dải góc thực tế so với góc lệnh**.
- USART2 và DMA nhận UART đã được cấu hình ở mức ngoại vi; firmware thử nghiệm hiện dùng cách nhận polling ở USART1, chưa triển khai giao thức điều khiển máy tính qua USART2.

**Vấn đề đang xử lý**

Với dải xung `1000–2000 µs`, góc quay thực tế của servo nhỏ hơn góc điều khiển kỳ vọng. Khi thử `500–2500 µs`, servo quay mạnh hơn và có nguy cơ chạm giới hạn cơ khí. Do đó **chưa chốt thông số PWM cuối cùng**.

Cần xác định giới hạn xung an toàn **riêng cho từng servo**, đo góc quay thực tế, đặt giới hạn cơ khí theo từng khớp và tinh chỉnh tốc độ chuyển động. Không coi giá trị góc 0–180 trong lệnh là bảo đảm servo quay đúng 180° vật lý.

### Công việc còn lại và thứ tự triển khai

#### Giai đoạn 1: Hoàn thiện điều khiển phần cứng

- [ ] Đo và hiệu chuẩn xung PWM tối thiểu, trung tâm, tối đa của **từng** servo; tránh tiếng rít, rung, nóng và điểm chặn cơ khí.
- [ ] Xác định chiều quay, góc gốc, giới hạn làm việc thực tế của từng khớp J1–J6 và cập nhật vào firmware.
- [ ] Bảo đảm nguồn cấp servo riêng đủ dòng, nối chung GND với STM32; thử ổn định khi nhiều khớp hoạt động.
- [ ] Hoàn thiện chuyển động mượt, giới hạn tốc độ và xử lý tình huống mất kết nối, kẹt khớp hoặc khởi động lệch tư thế.
- [ ] Kiểm tra khả năng gắp vật 100 g, 200 g, 300 g; ghi nhận độ võng, độ chính xác và nhiệt độ servo.

#### Giai đoạn 2: Giao diện điều khiển thủ công

- [ ] Thống nhất giao thức lệnh dùng chung cho giao diện và STM32, có phản hồi trạng thái và kiểm tra dữ liệu.
- [ ] Xây dựng giao diện điện thoại với **6 thanh trượt** và/hoặc joystick, gửi lệnh qua HC-06.
- [ ] Xây dựng giao diện máy tính; kết nối USART2 qua bộ USB–UART, kiểm thử cả hai chiều.
- [ ] Thêm nút Home, Stop, điều khiển riêng từng khớp và hiển thị góc lệnh.
- [ ] Kiểm thử giới hạn khớp và khả năng duy trì kết nối khi người dùng điều khiển liên tục.

#### Giai đoạn 3: Tự động thực hiện chuỗi thao tác

- [ ] Xây dựng cấu trúc lưu tư thế gồm góc 6 khớp và thời gian chuyển động.
- [ ] Viết chức năng Teach & Repeat: ghi tư thế, lưu một chuỗi, chạy lại, tạm dừng và dừng.
- [ ] Bổ sung nội suy chuyển động để thay đổi nhiều khớp phối hợp mà không giật.
- [ ] Tạo một kịch bản minh họa, ví dụ: về vị trí gốc → tiếp cận vật → kẹp → nâng → di chuyển → thả → trở về.
- [ ] Thử lặp nhiều chu kỳ; kiểm tra sai số vị trí và độ ổn định của nguồn, cơ cấu kẹp.

#### Giai đoạn 4: ROS 2 (nâng cao)

- [ ] Rà soát và hoàn thiện các script `RobotArm/scr/` hiện có, kiểm tra các topic và định dạng dữ liệu đầu ra.
- [ ] Hoàn thiện node cầu nối ROS 2 ↔ UART; điều chỉnh cho đúng cổng, baudrate và giao thức của STM32F411 hiện tại.
- [ ] Thiết lập mô hình robot/URDF với thông số hình học đo từ cơ cấu thực tế, cấu hình TF và giới hạn khớp.
- [ ] Kiểm tra động học thuận, động học nghịch và hiển thị trong RViz; đối chiếu góc tính toán với chuyển động thật.
- [ ] Tích hợp điều khiển vị trí đầu công tác và thử nghiệm chuỗi thao tác từ ROS 2.

### Cấu trúc repository

```text
VDK-Canh-tay-robot/
├── 3D PARTS/              # Các file STL của cơ cấu robot và gripper
├── RobotArm_6DOF/         # Dự án STM32F411CEU6 đang phát triển
│   ├── RobotArm_6DOF.ioc # Cấu hình ngoại vi STM32CubeMX
│   └── Core/Src/main.c    # Firmware thử điều khiển servo qua Bluetooth
├── RobotArm/              # Dự án STM32F103 và mã nghiên cứu trước đó
│   └── scr/               # Các script Python/ROS 2 thử nghiệm
└── README.md
```

> **Lưu ý về phiên bản:** Mã `main.c` đang có trong bản làm việc repository hiện đặt giới hạn thử `80–100°` cho cả 6 khớp. Bản mã người thực hiện gửi riêng để rà soát đã mở giới hạn J1–J5 đến `0–180°`, J6 đến `45–125°`. Các giá trị này là cấu hình thử nghiệm, không phải giới hạn cơ khí đã hiệu chuẩn. Khi cập nhật firmware trong repository, cần cập nhật README cho khớp phiên bản.

---

## Test từng khớp servo qua Bluetooth Serial (STM32F411CEU6)

Firmware: `RobotArm_6DOF/Core/Src/main.c` (STM32CubeIDE/Keil HAL project).

### Ket noi

| STM32 | Ket noi |
| --- | --- |
| PA9 (USART1_TX) | RXD cua module Bluetooth UART |
| PA10 (USART1_RX) | TXD cua module Bluetooth UART |
| GND | GND Bluetooth va GND nguon servo |
| PA6 (TIM3_CH1) | Servo 1 signal |
| PA7 (TIM3_CH2) | Servo 2 signal |
| PB0 (TIM3_CH3) | Servo 3 signal |
| PB1 (TIM3_CH4) | Servo 4 signal |
| PB6 (TIM4_CH1) | Servo 5 signal |
| PB7 (TIM4_CH2) | Servo 6 signal |

Dung nguon servo rieng phu hop dien ap/dong dien cua servo; **khong cap nguon cho 6 servo tu chan 3.3 V STM32**. Noi chung GND. Kiem tra dien ap logic UART va muc 3.3 V cua STM32; dung chuyen muc neu module xuat TX 5 V khong tuong thich. Canh tay nen duoc thao lien ket/tranh tai va de thoang co khi test lan dau.

Module HC-06 cua du an duoc xac nhan dang chay **9600 baud, 8N1, khong hardware flow control**. Firmware ap dung `BLUETOOTH_BAUDRATE = 9600U` sau `MX_USART1_UART_Init()`, du `.ioc` van co the dang hien thi 115200. Khi sinh lai ma tu CubeMX, can giu nguyen phan `USER CODE` co khoi cau hinh baudrate nay, hoac doi USART1 trong `.ioc` sang 9600 cho dong bo. HC-06 su dung Bluetooth Classic Serial (SPP).

### Gui lenh

Mo ung dung Bluetooth Serial Terminal, ket noi module. Gui moi lenh rieng biet; **khong bat buoc ky tu xuong dong**. Firmware xu ly khi gap CR/LF hoac khi khong co byte moi trong 100 ms. Vi du:

```text
PING
HELP
STATUS
J1 90
J1 +1
J1 -1
J2 95
STOP
OFF 1
OFF ALL
```

| Lenh | Tac dung |
| --- | --- |
| `J1 90` | Dat muc tieu servo 1 den 90 do (goc tuyet doi) |
| `J1 +1` | Tang muc tieu servo 1 them 1 do |
| `J1 -1` | Giam muc tieu servo 1 di 1 do |
| `J2 95` | Dat muc tieu servo 2 den 95 do; cac khop khac dung di chuyen va giu vi tri |
| `STATUS` | Xem goc dieu khien hien tai, muc tieu va trang thai PWM |
| `STOP` | Dung cap nhat goc va tiep tuc giu luc servo; khong ngat nguon |
| `OFF 1` | Dung xung PWM servo 1, servo khong con duoc giu vi tri |
| `OFF ALL` | Dung xung PWM tat ca servo, khong con giu vi tri |

So khop hop le: 1 den 6. De kiem tra an toan, **gioi han goc mac dinh cua tung khop la 80 den 100 do**. Thu lan luot `J1 90`, `J1 +1`, `J1 -1`, sau do `J2 90`, ... `J6 90`. Chi mo rong `servo_min_deg[]` va `servo_max_deg[]` sau khi xac dinh gioi han co khi. Goc thuc te phu thuoc model servo va hieu chuan, khong duoc suy ra tu gia tri dieu khien.

Moi lan gui lenh chi thay doi **mot khop**. Servo duoc ra lenh se tien dan 1 do moi 20 ms (khoang 50 do/giay tren *gia tri PWM*); cac khop da bat PWM van giu goc cu. Firmware **khong tu bat PWM cho toan bo 6 servo khi cap nguon**. Khi lan dau ra lenh mot servo, firmware bat PWM tu moc 90 do, sau do di den goc yeu cau. Vi servo thuong khong bao ve goc thuc ve, lan kich hoat dau tien **co the gay dich chuyen dot ngot** neu truc dang o vi tri khac 90 do. Nen canh chinh ban dau va test gia tri `J1 90`, `J1 +1`, `J1 -1` khi chua lap co cau, sau do test tung khop mot.

### Cai dat co ban

Trong `Core/Src/main.c` co the sua:

* `SERVO_MIN_PULSE_US` / `SERVO_MAX_PULSE_US`: mac dinh 1000 va 2000 microgiay. Goc 0..180 la quy uoc noi suy tu khoang xung nay, khong phai thong so bao dam cho moi servo.
* `servo_min_deg[]` / `servo_max_deg[]`: gioi han goc rieng tung khop theo co khi thuc te. **Cai gioi han co khi truoc khi lap vao canh tay.**
* `SERVO_UPDATE_PERIOD_MS`: mac dinh 20 ms, moi lan tang/giam 1 do.

TIM3/TIM4 dang chay PWM 50 Hz, tick 1 MHz. USART2 (PA2/PA3) duoc giu nguyen cho laptop va chua duoc su dung boi giao thuc test nay. Phan nhan Bluetooth su dung **HAL_UART_Receive polling** giong firmware BLE echo da thanh cong. Cau hinh DMA trong `.ioc` duoc giu nguyen nhung firmware nay khong start RX DMA.

### Kiem tra

Nap firmware bang STM32CubeIDE/Keil. Cap nguon STM32 va servo, sau do gui `PING` phai nhan `PONG`; gui `STATUS` de xac nhan tat ca khop dang `OFF` luc moi khoi dong. Gui `J1 90` de bat rieng khop 1, roi `J1 +1` va `J1 -1`. Neu khong co tra loi, kiem tra TX/RX dau cheo, chung GND va `BLUETOOTH_BAUDRATE` co khop voi firmware BLE echo da test thanh cong hay khong.
>>>>>>> Stashed changes
