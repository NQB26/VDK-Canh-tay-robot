
"""Test tung khop canh tay robot 6DOF bang goc gui qua cong serial / Bluetooth.

Chi can pyserial (khong can aiohttp, khong can server.py). Dong server.py truoc
khi chay vi hai chuong trinh khong dung chung duoc mot cong COM.

Vi du:
    py test_joints.py --list                    xem cac cong COM
    py test_joints.py COM7                      quet thu lan luot tung khop (bien do nho +-20 do)
    py test_joints.py COM7 --joint 3            chi test khop 3
    py test_joints.py COM7 --delta 40 --speed 60
    py test_joints.py COM7 -i                   che do go lenh bang tay
    py test_joints.py COM7 --home               dua tat ca khop ve 90 do

Lenh gui cho firmware (ket thuc bang '\\n'):
    S<n>:<goc>   dat khop n (1..6)        vd  S3:120
    V<toc_do>    toc do toi da do/giay     vd  V40
    a1,..,a6     dat ca 6 khop             vd  90,90,120,90,90,90
"""
import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("Thieu pyserial. Cai bang:  py -m pip install pyserial")

NAMES = ["De (xoay)", "Vai", "Khuyu", "Co tay (gap)", "Co tay (xoay)", "Kep"]
HOME = 90


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Arm:
    def __init__(self, ser, lo, hi):
        self.ser = ser
        self.lo, self.hi = lo, hi
        self.pos = [float(HOME)] * 6      # vi tri muc tieu cuoi cung da gui
        self.speed = 40.0

    def send(self, text):
        print(f"  >> {text}")
        self.ser.write((text + "\n").encode("ascii"))
        self.ser.flush()

    def set_speed(self, dps):
        self.speed = float(dps)
        self.send(f"V{int(dps)}")

    def move(self, joint, angle, wait=True):
        """joint = 1..6. Tra ve thoi gian cho (giay) de servo toi noi."""
        angle = clamp(angle, self.lo, self.hi)
        dist = abs(angle - self.pos[joint - 1])
        self.pos[joint - 1] = angle
        self.send(f"S{joint}:{angle:g}")
        t = dist / self.speed + 0.3
        if wait:
            time.sleep(t)
        return t

    def move_all(self, angles):
        angles = [clamp(a, self.lo, self.hi) for a in angles]
        dist = max(abs(a - p) for a, p in zip(angles, self.pos))
        self.pos = [float(a) for a in angles]
        self.send(",".join(f"{a:g}" for a in angles))
        time.sleep(dist / self.speed + 0.3)

    def home(self):
        self.move_all([HOME] * 6)


def sweep_joint(arm, joint, delta):
    """90 -> 90+delta -> 90-delta -> 90"""
    print(f"\n=== Khop {joint}: {NAMES[joint - 1]} (bien do +-{delta} do) ===")
    for target in (HOME + delta, HOME - delta, HOME):
        print(f"  khop {joint} -> {clamp(target, arm.lo, arm.hi):g} do")
        arm.move(joint, target)
    time.sleep(0.3)


def run_sweep(arm, joints, delta, pause):
    for j in joints:
        sweep_joint(arm, j, delta)
        time.sleep(pause)


def run_interactive(arm):
    print("""
Go lenh roi Enter:
  2 120            dat khop 2 = 120 do          (khop 1..6)
  all 90,45,120,90,90,30   dat ca 6 khop
  home             ve 90 do het
  v 60             doi toc do toi da (do/giay)
  sweep 3          quet thu khop 3
  q                thoat
""")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        parts = line.replace(",", " ").split()
        cmd = parts[0].lower()
        try:
            if cmd in ("q", "quit", "exit"):
                return
            if cmd == "home":
                arm.home()
            elif cmd == "v" and len(parts) == 2:
                arm.set_speed(float(parts[1]))
            elif cmd == "sweep" and len(parts) == 2 and 1 <= int(parts[1]) <= 6:
                sweep_joint(arm, int(parts[1]), 20)
            elif cmd == "all" and len(parts) == 7:
                arm.move_all([float(x) for x in parts[1:]])
            elif len(parts) == 2 and 1 <= int(parts[0]) <= 6:
                arm.move(int(parts[0]), float(parts[1]), wait=False)
            else:
                print("  Khong hieu lenh.")
        except ValueError:
            print("  Gia tri khong hop le.")


def main():
    ap = argparse.ArgumentParser(description="Test cac khop canh tay bang serial/Bluetooth")
    ap.add_argument("port", nargs="?", help="cong COM, vd COM7")
    ap.add_argument("--baud", type=int, default=115200,
                    help="toc do cong COM (mac dinh 115200, khop USART2 cua firmware). "
                         "Voi cong Bluetooth ao thi gia tri nay gan nhu khong anh huong")
    ap.add_argument("--list", action="store_true", help="liet ke cong COM roi thoat")
    ap.add_argument("--joint", type=int, choices=range(1, 7), metavar="1-6", help="chi test mot khop")
    ap.add_argument("--delta", type=float, default=20, help="bien do quet quanh 90 do (mac dinh 20)")
    ap.add_argument("--speed", type=float, default=40, help="toc do toi da do/giay (mac dinh 40)")
    ap.add_argument("--min", type=float, default=0, dest="lo", help="gioi han goc duoi (mac dinh 0)")
    ap.add_argument("--max", type=float, default=180, dest="hi", help="gioi han goc tren (mac dinh 180)")
    ap.add_argument("--pause", type=float, default=1.0, help="nghi giua cac khop (giay)")
    ap.add_argument("-i", "--interactive", action="store_true", help="go lenh bang tay")
    ap.add_argument("--home", action="store_true", help="chi dua ve 90 do roi thoat")
    ap.add_argument("-y", "--yes", action="store_true", help="khong hoi xac nhan truoc khi chay")
    args = ap.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return
    if not args.port:
        ap.error("can ten cong COM (xem bang --list)")
    if args.lo >= args.hi:
        ap.error("--min phai nho hon --max")
    delta = abs(args.delta)

    print(f"Dang mo {args.port} (cong Bluetooth co the mat 10-20 giay lan dau)...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=2)
    except (serial.SerialException, OSError) as exc:
        sys.exit(f"Khong mo duoc {args.port}: {exc}\n"
                 "Kiem tra: dong server.py/Keil/terminal dang giu cong, thu cong COM con lai cua module.")
    print("Da mo cong.")

    arm = Arm(ser, args.lo, args.hi)
    try:
        time.sleep(0.5)
        arm.set_speed(args.speed)

        if args.home:
            arm.home()
            return

        if not args.yes:
            print("\nCANH BAO: canh tay se chuyen dong. De no o cho thoang va giu tay xa cac khop.")
            if not args.interactive:
                print(f"Se quet khop {args.joint or '1..6'} quanh {HOME} do, bien do +-{delta:g} do, "
                      f"toc do {args.speed:g} do/giay.")
            input("Enter de bat dau (Ctrl+C de huy)... ")

        if args.interactive:
            run_interactive(arm)
        else:
            print("\nDua ve 90 do truoc...")
            arm.home()
            run_sweep(arm, [args.joint] if args.joint else range(1, 7), delta, args.pause)
            print("\nXong. Khop nao khong quay: kiem tra day servo, nguon servo va gioi han goc.")
    except KeyboardInterrupt:
        print("\nDa dung. Canh tay giu nguyen o muc tieu cuoi cung.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
