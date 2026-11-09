#!/usr/bin/env python3
"""
Raspberry Pi 5 tren drone - Nhan dien ArUco, tich hop CLAHE & Bo loc EMA, gui UDP.
"""
import argparse
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np

# ====== CAU HINH MAC DINH ======
LAPTOP_IP = "192.168.31.190"
UDP_PORT = 5005
STREAM_PORT = 8000
ARUCO_DICT = "auto"

CANDIDATES = [
    "DICT_4X4_1000",
    "DICT_5X5_1000",
    "DICT_6X6_1000",
    "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
]
DETECT_W, DETECT_H = 1280, 720
RESEND_INTERVAL = 0.5
MIN_FRAMES = 3
LOCK_FRAMES = 5
ECR = 0.6
STREAM_FPS = 20
STREAM_WIDTH = 640
JPEG_QUALITY = 70

MARKER_NAMES = {
    13: "Vi tri ID 13",
    17: "Vi tri ID 17",
    37: "Vi tri ID 37",
}

# ====== BO LOC EMA (LAM MUOT TOA DO) ======
class EMAFilter:
    def __init__(self, alpha=0.3):
        """
        alpha: Trong so (0 < alpha <= 1). 
        Gia tri cang nho -> cang muot (nhưng se hoi co do tre nhat dinh).
        Gia tri cang lon -> cang bam sat thuc te (nhung de bi rung).
        """
        self.alpha = alpha
        self.value = None

    def update(self, new_val):
        if self.value is None:
            self.value = new_val
        else:
            self.value = self.alpha * new_val + (1 - self.alpha) * self.value
        return self.value

# ====== PHAT VIDEO MJPEG + TRANG THAI ======
PAGE = b"""<html><head><meta charset="utf-8"><title>ArUco Tracker</title></head>
<body style="margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center">
<img src="/stream" style="width:100%;max-width:960px">
<div id="s" style="padding:10px;font-size:18px">dang ket noi...</div>
<script>
setInterval(async () => {
  try {
    const j = await (await fetch('/status')).json();
    document.getElementById('s').textContent =
      'Tu dien: ' + j.dict + '  |  FPS: ' + j.fps + '  |  ID thay: ' +
      (j.ids.length ? j.ids.join(', ') : '-');
  } catch (e) {}
}, 500);
</script></body></html>"""

class StreamState:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.clients = 0
        self.info = {"dict": "...", "fps": 0, "ids": []}

    def publish(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def set_info(self, **kw):
        with self.cond:
            self.info.update(kw)

def start_stream_server(state, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def _send(self, ctype, body):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send("text/html; charset=utf-8", PAGE)
            elif self.path == "/status":
                with state.cond:
                    body = json.dumps(state.info).encode()
                self._send("application/json", body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with state.cond:
                    state.clients += 1
                last = -1
                try:
                    while True:
                        with state.cond:
                            state.cond.wait_for(
                                lambda: state.jpeg is not None and state.seq != last,
                                timeout=5,
                            )
                            if state.jpeg is None or state.seq == last:
                                continue
                            jpeg, last = state.jpeg, state.seq
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except OSError:
                    pass
                finally:
                    with state.cond:
                        state.clients -= 1
            else:
                self.send_error(404)

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server

def local_ip_towards(target_ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((target_ip, 9))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<IP_Pi>"

# ====== CAMERA CSI (PICAMERA2) ======
class CsiCamera:
    def __init__(self, width, height):
        from picamera2 import Picamera2
        if not Picamera2.global_camera_info():
            raise RuntimeError("Khong tim thay camera CSI")
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.cam.configure(cfg)
        try:
            # Khoa 30 FPS va tang toc do man trập (exposure) de chong nhoehinh khi bay
            self.cam.set_controls({
                "AfMode": 2, 
                "FrameDurationLimits": (33333, 33333),
                "ExposureTime": 5000  # ~1/200s giup hinh anh net khi rung
            })
        except Exception:
            pass
        self.cam.start()
        time.sleep(0.5)

    def read(self):
        try:
            return True, self.cam.capture_array()
        except Exception:
            return False, None

    def release(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass

def open_camera(args):
    while True:
        try:
            cam = CsiCamera(args.width, args.height)
            print("[CAMERA] Dang dung camera CSI (Picamera2)", flush=True)
            return cam
        except Exception as e:
            print(f"[CAMERA] Loi: {e}. Thu lai sau 3 giay...", flush=True)
            time.sleep(3)

# ====== ARUCO DETECTOR & CLAHE ======
def make_detector(dict_name, ecr=ECR):
    if not hasattr(cv2.aruco, dict_name):
        sys.exit(f"[LOI] Khong co tu dien ArUco '{dict_name}'")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, "ArucoDetector"):
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.errorCorrectionRate = ecr
        return cv2.aruco.ArucoDetector(dictionary, p).detectMarkers
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.errorCorrectionRate = ecr
    return lambda g: cv2.aruco.detectMarkers(g, dictionary, parameters=p)

class DictLocker:
    def __init__(self, names, ecr, need):
        self.dets = {n: make_detector(n, ecr) for n in names}
        self.need = need
        self.streak = {}
        self.locked = None

    def scan(self, gray):
        seen = set()
        for name, det in self.dets.items():
            _, ids, _ = det(gray)
            if ids is not None:
                for i in ids.flatten():
                    seen.add((name, int(i)))
        for k in list(self.streak):
            if k not in seen:
                del self.streak[k]
        for k in seen:
            self.streak[k] = self.streak.get(k, 0) + 1
            if self.streak[k] >= self.need and self.locked is None:
                self.locked = k[0]
                return

    @property
    def detect(self):
        return self.dets[self.locked]

def annotate(frame, corners, ids, banner=None):
    h, w = frame.shape[:2]
    scale = max(0.5, 0.6 * w / 640)
    thick = max(1, int(round(2 * w / 640)))
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            x, y = c[0][0]
            name = MARKER_NAMES.get(i)
            label = f"ID {i}: {name}" if name else f"ID {i}"
            cv2.putText(frame, label, (int(x), max(int(y) - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), thick)
    if banner:
        cv2.putText(frame, banner, (10, int(30 * scale / 0.6)),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 200, 255), thick)

def build_message(marker_id, fmt, smoothed_pos=None):
    if fmt == "json":
        return json.dumps({
            "id": marker_id,
            "name": MARKER_NAMES.get(marker_id, ""),
            "pos": smoothed_pos, # Truyen toa do da loc EMA [X, Y, Z]
            "t": round(time.time(), 3),
        }).encode()
    return str(marker_id).encode()

# ====== MAIN ======
def main():
    ap = argparse.ArgumentParser(description="Nhan dien ArUco voi CLAHE & EMA Filter")
    ap.add_argument("--ip", default=LAPTOP_IP)
    ap.add_argument("--port", type=int, default=UDP_PORT)
    ap.add_argument("--dict", default=ARUCO_DICT)
    ap.add_argument("--width", type=int, default=DETECT_W)
    ap.add_argument("--height", type=int, default=DETECT_H)
    ap.add_argument("--stream-port", type=int, default=STREAM_PORT)
    args = ap.parse_args()

    cam = open_camera(args)

    locker = None
    detect = None
    if args.dict.lower() == "auto":
        locker = DictLocker(CANDIDATES, ECR, LOCK_FRAMES)
        print(f"[AUTO] Dang do loai ma trong {len(CANDIDATES)} tu dien...", flush=True)
    else:
        detect = make_detector(args.dict, ECR)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_sent = {}
    streak = {}

    # Khoi tao bo loc EMA cho toa do X, Y, Z (alpha = 0.3 giup muot ma, it giat)
    ema_x = EMAFilter(alpha=0.3)
    ema_y = EMAFilter(alpha=0.3)
    ema_z = EMAFilter(alpha=0.3)

    # Khoi tao bo xu ly anh CLAHE chong loasang
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    state = StreamState()
    start_stream_server(state, args.stream_port)
    print(f"[VIDEO] Xem truc tuyen: http://{local_ip_towards(args.ip)}:{args.stream_port}", flush=True)

    last_stream = 0.0
    last_info = 0.0
    fps = 0.0
    t_prev = time.time()
    fails = 0

    try:
        while True:
            ok, frame = cam.read()
            if not ok or frame is None:
                fails += 1
                if fails >= 30:
                    cam.release()
                    cam = open_camera(args)
                    fails = 0
                time.sleep(0.05)
                continue
            fails = 0

            now = time.time()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            # Chuyen sang anh xám va ap dung CLAHE chong loa sang ngoai troi
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = clahe.apply(gray)

            # --- Chon tu dien ---
            searching = False
            if locker is not None and locker.locked is None:
                locker.scan(gray)
                if locker.locked:
                    detect = locker.detect
            if detect is None:
                searching = True
                corners, ids = (), None
            else:
                corners, ids, _ = detect(gray)

            # --- Xu ly toa do & Loc EMA ---
            seen = set()
            smoothed_coords = None
            if ids is not None:
                # Gia su gia tr tri dung Camera Matrix gia dinh hoac uoc luong thoi diem dau
                # Neu co Camera Calibration rieng, thay ma tran vao day.
                cameraMatrix = np.array([[800, 0, DETECT_W/2], [0, 800, DETECT_H/2], [0, 0, 1]], dtype=float)
                distCoeffs = np.zeros((5, 1))

                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)
                    seen.add(marker_id)

                    # Tinh toan vector xoay va tinh tien (tvec) cua ArUco
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], 0.05, cameraMatrix, distCoeffs
                    )
                    x = tvecs[0][0][0]
                    y = tvecs[0][0][1]
                    z = tvecs[0][0][2]

                    # Ap dung bo loc EMA lam muot
                    s_x = ema_x.update(x)
                    s_y = ema_y.update(y)
                    s_z = ema_z.update(z)
                    smoothed_coords = [round(float(s_x), 3), round(float(s_y), 3), round(float(s_z), 3)]

                    streak[marker_id] = streak.get(marker_id, 0) + 1
                    if (streak[marker_id] >= MIN_FRAMES and now - last_sent.get(marker_id, 0) >= RESEND_INTERVAL):
                        try:
                            sock.sendto(build_message(marker_id, "json", smoothed_coords), (args.ip, args.port))
                            print(f"Gui ID {marker_id} | Toa do mượt (X,Y,Z): {smoothed_coords}", flush=True)
                        except OSError:
                            pass
                        last_sent[marker_id] = now

            for k in list(streak):
                if k not in seen:
                    streak[k] = 0

            # --- Cap nhat trang thai web ---
            if now - last_info >= 0.25:
                dict_name = (detect and (locker.locked if locker else args.dict)) or "dang do..."
                state.set_info(dict=dict_name, fps=round(fps, 1), ids=sorted(seen))
                last_info = now

            # --- Stream video ---
            if state.clients > 0 and now - last_stream >= 1.0 / STREAM_FPS:
                banner = "Dang tim ma ArUco..." if searching else None
                annotate(frame, corners, ids, banner)
                out = frame
                if frame.shape[1] > STREAM_WIDTH:
                    s = STREAM_WIDTH / frame.shape[1]
                    out = cv2.resize(frame, (STREAM_WIDTH, int(frame.shape[0] * s)), interpolation=cv2.INTER_AREA)
                okj, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if okj:
                    state.publish(buf.tobytes())
                    last_stream = now

    except KeyboardInterrupt:
        print("\nDung chuong trinh.")
    finally:
        cam.release()
        sock.close()

if __name__ == "__main__":
    main()