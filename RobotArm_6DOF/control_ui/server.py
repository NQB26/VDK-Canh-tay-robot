"""Cau noi Web UI <-> STM32 (RobotArm_6DOF).

Chay tren laptop / Raspberry Pi noi voi STM32 qua USB-UART (USART2) hoac
module BLE/Bluetooth serial. Phuc vu trang web dieu khien (thanh truot +
joystick) cho trinh duyet tren may tinh hoac dien thoai trong cung mang Wi-Fi.

    python server.py --serial COM5            (Windows)
    python server.py --serial /dev/ttyUSB0    (Linux)
    python server.py --sim                    (khong can phan cung, de thu giao dien)
"""
import argparse
import asyncio
import hmac
import json
import logging
import math
import socket
import sys
from pathlib import Path

from aiohttp import WSMsgType, web

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # chi bat buoc khi khong dung --sim
    serial = None

NUM_SERVO = 6
HOME = 90.0
SEND_HZ = 50                 # gioi han toc do gui xuong STM32
SPEED_MIN, SPEED_MAX = 10, 360
STATIC_DIR = Path(__file__).parent / "static"

log = logging.getLogger("robotarm")


class Arm:
    """Giu trang thai muc tieu cua canh tay va ghi lenh xuong cong serial."""

    def __init__(self, port, baud, sim):
        self.port = port
        self.baud = baud
        self.sim = sim
        self.angles = [HOME] * NUM_SERVO
        self.speed = 90
        self.dirty = False        # co goc moi chua gui
        self.speed_dirty = False
        self.ser = None

    @property
    def connected(self):
        return self.sim or (self.ser is not None and self.ser.is_open)

    def _open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.2)
        # Sau khi (noi lai) cong, dong bo lai toc do va vi tri hien tai
        self.speed_dirty = True
        self.dirty = True
        log.info("Da mo %s @ %d", self.port, self.baud)

    async def try_open(self):
        if self.sim or self.connected:
            return
        try:
            await asyncio.get_running_loop().run_in_executor(None, self._open)
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            log.warning("Chua mo duoc %s: %s (thu lai sau 2 giay)", self.port, exc)

    def _write(self, text):
        if self.sim:
            log.debug("SIM >> %r", text)
            return True
        try:
            self.ser.write(text.encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            log.warning("Mat ket noi serial: %s", exc)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False

    def flush(self):
        """Gui lenh dang cho (neu co). Goi dinh ky, nen luon <= SEND_HZ lenh/giay."""
        if not self.connected:
            return
        if self.speed_dirty:
            if not self._write(f"V{self.speed}\n"):
                return
            self.speed_dirty = False
        if self.dirty:
            line = ",".join(f"{a:.1f}" for a in self.angles) + "\n"
            if self._write(line):
                self.dirty = False


def parse_angles(raw):
    """Kiem tra du lieu tu client; tra ve list 6 goc da kep 0..180 hoac None."""
    if not isinstance(raw, list) or len(raw) != NUM_SERVO:
        return None
    out = []
    for v in raw:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            return None
        out.append(min(180.0, max(0.0, float(v))))
    return out


def state_msg(arm, clients):
    return {
        "type": "state",
        "a": arm.angles,
        "speed": arm.speed,
        "serial": arm.connected,
        "sim": arm.sim,
        "port": arm.port if not arm.sim else "SIM",
        "clients": len(clients),
    }


async def broadcast(app, msg, skip=None):
    for ws in list(app["clients"]):
        if ws is not skip and not ws.closed:
            try:
                await ws.send_json(msg)
            except ConnectionResetError:
                pass


async def index(request):
    return web.FileResponse(STATIC_DIR / "index.html")


async def ws_handler(request):
    app = request.app
    token = app["token"]
    if token and not hmac.compare_digest(request.query.get("token", ""), token):
        raise web.HTTPUnauthorized(text="Sai token")

    ws = web.WebSocketResponse(heartbeat=15, max_msg_size=4096)
    await ws.prepare(request)
    arm = app["arm"]
    app["clients"].add(ws)
    log.info("Client ket noi (%d dang mo)", len(app["clients"]))
    await ws.send_json(state_msg(arm, app["clients"]))

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except ValueError:
                continue
            if not isinstance(data, dict):
                continue

            kind = data.get("type")
            if kind == "angles":
                angles = parse_angles(data.get("a"))
                if angles is not None:
                    arm.angles = angles
                    arm.dirty = True
                    await broadcast(app, {"type": "angles", "a": angles}, skip=ws)
            elif kind == "speed":
                v = data.get("v")
                if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
                    arm.speed = int(min(SPEED_MAX, max(SPEED_MIN, v)))
                    arm.speed_dirty = True
                    await broadcast(app, {"type": "speed", "v": arm.speed}, skip=ws)
    finally:
        app["clients"].discard(ws)
        log.info("Client ngat ket noi (%d con lai)", len(app["clients"]))
    return ws


async def sender_loop(app):
    arm = app["arm"]
    last_try = 0.0
    loop = asyncio.get_running_loop()
    while True:
        await asyncio.sleep(1.0 / SEND_HZ)
        if not arm.connected and loop.time() - last_try >= 2.0:
            last_try = loop.time()
            await arm.try_open()
        arm.flush()


async def status_loop(app):
    while True:
        await asyncio.sleep(1.0)
        await broadcast(app, state_msg(app["arm"], app["clients"]) | {"type": "status"})


async def start_tasks(app):
    app["tasks"] = [
        asyncio.create_task(sender_loop(app)),
        asyncio.create_task(status_loop(app)),
    ]


async def stop_tasks(app):
    for t in app["tasks"]:
        t.cancel()
    await asyncio.gather(*app["tasks"], return_exceptions=True)
    arm = app["arm"]
    if arm.ser is not None:
        arm.ser.close()


def lan_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))   # khong gui goi tin nao
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def pick_serial_port(requested):
    if requested:
        return requested
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if len(ports) == 1:
        return ports[0]
    sys.exit(
        "Hay chon cong serial bang --serial. Cac cong hien co: "
        + (", ".join(ports) if ports else "(khong thay cong nao)")
    )


def main():
    ap = argparse.ArgumentParser(description="Web UI dieu khien canh tay robot 6DOF")
    ap.add_argument("--serial", help="cong serial noi STM32, vd COM5 hoac /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="0.0.0.0", help="dia chi lang nghe (mac dinh: ca mang LAN)")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--token", help="neu dat: client phai mo trang voi ?token=<gia tri>")
    ap.add_argument("--sim", action="store_true", help="che do mo phong, khong can STM32")
    ap.add_argument("--list", action="store_true", help="liet ke cong serial roi thoat")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.list:
        if serial is None:
            sys.exit("Thieu pyserial: pip install -r requirements.txt")
        for p in serial.tools.list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return
    if not args.sim and serial is None:
        sys.exit("Thieu pyserial: pip install -r requirements.txt (hoac dung --sim)")

    port = None if args.sim else pick_serial_port(args.serial)

    app = web.Application()
    app["arm"] = Arm(port, args.baud, args.sim)
    app["clients"] = set()
    app["token"] = args.token
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.on_startup.append(start_tasks)
    app.on_cleanup.append(stop_tasks)

    suffix = f"/?token={args.token}" if args.token else "/"
    print(f"\n  May tinh : http://localhost:{args.http_port}{suffix}")
    print(f"  Dien thoai (cung Wi-Fi): http://{lan_ip()}:{args.http_port}{suffix}\n")
    web.run_app(app, host=args.host, port=args.http_port, print=None)


if __name__ == "__main__":
    main()
