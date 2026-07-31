"""
K230 steel-ball recognition + UART3 + Wi-Fi AP/RTSP.

This is a standalone script.  It contains the detector, UART protocol and
Wi-Fi/RTSP service; main_256.py is not required.

Wi-Fi:
    SSID: K230-RTSP
    PASS: 12345678
    RTSP: rtsp://192.168.4.1:8554/k230 (use the printed IP if different)
"""

import os
import gc
import time
import uctypes
import _thread
import image
import multimedia as mm

from media.vencoder import *
from media.sensor import *
from media.display import *
from media.media import *

from libs.PipeLine import ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import letterbox_pad_param
import nncase_runtime as nn
import ulab.numpy as np
import aidemo


KMODEL_PATH = "/sdcard/steel_ball_v5_256.kmodel"
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [256, 256]
SENSOR_ID = 2
RGB888P_SIZE = [640, 480]
H_MIRROR = True
V_FLIP = True
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3
PRINT_EVERY_N_FRAMES = 10

UART_ENABLED = True
UART_ID = 3
UART_BAUDRATE = 115200
UART_TX_PIN = 50
UART_RX_PIN = 51
UART_SEND_INTERVAL = 1
UART_DEBUG = False

CENTER_X = 345
CENTER_Y = 192
OFFSET_SCALE = 1.0
ALIGN_DEAD_ZONE = 2


class SteelBallDetector(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 confidence_threshold=0.5, nms_threshold=0.3,
                 rgb888p_size=None, display_size=None, debug_mode=0):
        if rgb888p_size is None:
            rgb888p_size = [224, 224]
        if display_size is None:
            display_size = [800, 480]
        super().__init__(
            kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.class_id = LABELS
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [
            ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [
            ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self._letterbox_ratio = 1.0
        self._pad_top = 0
        self._pad_left = 0
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = (
                input_image_size if input_image_size else self.rgb888p_size)
            top, bottom, left, right, ratio = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size)
            self._letterbox_ratio = ratio
            self._pad_top = top
            self._pad_left = left
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [104, 117, 123])
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1],
                 self.model_input_size[0]])

    def postprocess(self, results):
        det_res = []
        with ScopedTiming("postprocess", self.debug_mode > 0):
            data = results[0][0]
            ratio = self._letterbox_ratio
            top = self._pad_top
            left = self._pad_left
            for i in range(data.shape[1]):
                score = data[4, i]
                if score > self.confidence_threshold:
                    x = (data[0, i] - left) / ratio
                    y = (data[1, i] - top) / ratio
                    w = data[2, i] / ratio
                    h = data[3, i] / ratio
                    det_res.append([x, y, w, h, 0, score])
            if det_res:
                det_res = self._nms(det_res)
        return det_res

    def _nms(self, dets):
        dets.sort(key=lambda x: x[-1], reverse=True)
        keep = []
        while dets:
            best = dets.pop(0)
            keep.append(best)
            dets = [
                item for item in dets
                if self._iou(best, item) < self.nms_threshold]
        return keep

    @staticmethod
    def _iou(a, b):
        ax, ay, aw, ah = a[:4]
        bx, by, bw, bh = b[:4]
        ax1, ay1 = ax - aw / 2, ay - ah / 2
        ax2, ay2 = ax + aw / 2, ay + ah / 2
        bx1, by1 = bx - bw / 2, by - bh / 2
        bx2, by2 = bx + bw / 2, by + bh / 2
        ix = max(0, min(ax2, bx2) - max(ax1, bx1))
        iy = max(0, min(ay2, by2) - max(ay1, by1))
        inter = ix * iy
        if inter <= 0:
            return 0
        return inter / (
            max(1, aw * ah) + max(1, bw * bh) - inter)

    def draw_result(self, context, dets):
        context.osd_img.clear()
        for det in dets:
            x, y, w, h = map(lambda value: int(round(value, 0)), det[:4])
            x = x * self.display_size[0] // self.rgb888p_size[0]
            y = y * self.display_size[1] // self.rgb888p_size[1]
            w = w * self.display_size[0] // self.rgb888p_size[0]
            h = h * self.display_size[1] // self.rgb888p_size[1]
            context.osd_img.draw_rectangle(
                x - w // 2, y - h // 2, w, h,
                color=(255, 0, 255, 0), thickness=2)


def _get_center():
    return CENTER_X, CENTER_Y


def build_uart_msg(best_center, best_score, count, center_ref):
    if best_center is None:
        return "DX:0,DY:0,A0,D0"
    dx = int(round((best_center[0] - center_ref[0]) * OFFSET_SCALE))
    dy = int(round((best_center[1] - center_ref[1]) * OFFSET_SCALE))
    aligned = 1 if (
        abs(dx) <= ALIGN_DEAD_ZONE and
        abs(dy) <= ALIGN_DEAD_ZONE) else 0
    return "DX:%d,DY:%d,A%d,D1" % (dx, dy, aligned)


def init_uart():
    if not UART_ENABLED:
        return None
    try:
        from machine import UART, FPIOA
        fpioa = FPIOA()
        fpioa.set_function(UART_RX_PIN, FPIOA.UART3_RXD)
        fpioa.set_function(UART_TX_PIN, FPIOA.UART3_TXD)
        uart = UART(UART.UART3, UART_BAUDRATE)
        print("[steel_ball] UART{} TX=IO{} RX=IO{} OK {}bps".format(
            UART_ID, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE))
        return uart
    except Exception as e:
        import sys
        print("[steel_ball] UART init FAILED:", e)
        sys.print_exception(e)
        return None


def send_uart(uart, msg):
    if uart is None:
        return
    try:
        written = uart.write((msg + "\n").encode())
        if UART_DEBUG:
            print("[UART] wrote={} {}".format(written, msg))
    except Exception as e:
        import sys
        print("[steel_ball] UART error:", e)
        sys.print_exception(e)


def overlay_osd(dets, context, frame_index, fps):
    count = len(dets) if dets else 0
    best_score = -1.0
    best_center = None
    for det in dets:
        score = float(det[-1])
        if score > best_score:
            best_score = score
            best_center = (
                int(round(det[0])),
                int(round(det[1])))

    context.osd_img.draw_string_advanced(
        5, 5, 24, "steel: %d" % count,
        color=(255, 0, 255, 0))
    if best_center is not None:
        cx = (
            best_center[0] * context.display_size[0] //
            RGB888P_SIZE[0])
        cy = (
            best_center[1] * context.display_size[1] //
            RGB888P_SIZE[1])
        context.osd_img.draw_cross(
            cx, cy, color=(255, 255, 255, 0),
            size=14, thickness=3)
        context.osd_img.draw_string_advanced(
            5, 34, 20,
            "%d,%d (%.2f)" % (
                best_center[0], best_center[1], best_score),
            color=(255, 255, 255, 0))
    else:
        context.osd_img.draw_string_advanced(
            5, 34, 20, "NO BALL",
            color=(255, 255, 0, 0))

    if fps > 0:
        context.osd_img.draw_string_advanced(
            context.display_size[0] - 80,
            5, 24, "FPS:%d" % fps,
            color=(255, 0, 255, 0))
    if frame_index % PRINT_EVERY_N_FRAMES == 0:
        if best_center is None:
            print("[steel_ball] frame=%d count=0 fps=%d" % (
                frame_index, fps))
        else:
            print(
                "[steel_ball] frame=%d count=%d best=(%d,%d) "
                "s=%.3f fps=%d" % (
                    frame_index, count,
                    best_center[0], best_center[1],
                    best_score, fps))
    return best_center, best_score, count


AP_SSID = "K230-RTSP"
AP_PASS = "12345678"
RTSP_PORT = 8554
RTSP_SESSION = "k230"
STREAM_WIDTH = RGB888P_SIZE[0]
STREAM_HEIGHT = RGB888P_SIZE[1]
STREAM_FPS = 30
STREAM_BITRATE_KBPS = 1500


class SensorWifi:
    def __init__(self):
        import network

        self.running = False
        self.link = None
        self.sensor = None
        self.encoder = None
        self.server = None

        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(ssid=AP_SSID, key=AP_PASS)
        while self.ap.ifconfig()[0] == "0.0.0.0":
            os.exitpoint()
            time.sleep_ms(100)
        print("[WiFi] AP={} IP={}".format(
            AP_SSID, self.ap.ifconfig()[0]))

        # Channel 0 feeds H.264, channel 1 feeds the LCD, and channel 2
        # supplies RGB planar frames to the AI detector.
        MediaManager.init()
        self.sensor = Sensor(
            id=SENSOR_ID, width=STREAM_WIDTH, height=STREAM_HEIGHT, fps=90)
        self.sensor.reset()
        self.sensor.set_hmirror(H_MIRROR)
        self.sensor.set_vflip(V_FLIP)

        self.sensor.set_framesize(
            width=STREAM_WIDTH, height=STREAM_HEIGHT,
            alignment=12, chn=CAM_CHN_ID_0)
        self.sensor.set_pixformat(
            Sensor.YUV420SP, chn=CAM_CHN_ID_0)

        self.sensor.set_framesize(
            width=STREAM_WIDTH, height=STREAM_HEIGHT,
            chn=CAM_CHN_ID_1)
        self.sensor.set_pixformat(
            Sensor.YUV420SP, chn=CAM_CHN_ID_1)

        self.sensor.set_framesize(
            width=STREAM_WIDTH, height=STREAM_HEIGHT,
            chn=CAM_CHN_ID_2)
        self.sensor.set_pixformat(
            Sensor.RGBP888, chn=CAM_CHN_ID_2)

        self.venc_channel = VENC_CHN_ID_0
        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            self.venc_channel, 8, STREAM_WIDTH, STREAM_HEIGHT)
        self.link = MediaManager.link(
            self.sensor.bind_info(chn=CAM_CHN_ID_0)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_channel))
        attr = ChnAttrStr(
            Encoder.PAYLOAD_TYPE_H264,
            Encoder.H264_PROFILE_BASELINE,
            STREAM_WIDTH,
            STREAM_HEIGHT,
            bit_rate=STREAM_BITRATE_KBPS,
            gopLen=30,
            src_frame_rate=STREAM_FPS,
            dst_frame_rate=STREAM_FPS)
        self.encoder.Create(self.venc_channel, attr)

        Display.init(
            Display.ST7701, width=800, height=480, to_ide=False)
        Display.bind_layer(
            **self.sensor.bind_info(chn=CAM_CHN_ID_1),
            layer=Display.LAYER_VIDEO1)
        self.display_size = [800, 480]
        self.osd_img = image.Image(
            ALIGN_UP(self.display_size[0], 16),
            self.display_size[1],
            image.ARGB8888)
        self.osd_img.clear()

        self.server = mm.rtsp_server()
        self.server.rtspserver_init(RTSP_PORT)
        self.server.rtspserver_createsession(
            RTSP_SESSION, mm.multi_media_type.media_h264, False)
        print("[RTSP] rtsp://{}:{}/{}".format(
            self.ap.ifconfig()[0], RTSP_PORT, RTSP_SESSION))

    def start(self):
        self.server.rtspserver_start()
        self.encoder.Start(self.venc_channel)
        self.sensor.run()
        self.running = True
        _thread.start_new_thread(self._rtsp_loop, ())
        _thread.start_new_thread(self._beacon_loop, ())

    def get_frame(self):
        return self.sensor.snapshot(chn=CAM_CHN_ID_2)

    def show_image(self):
        Display.show_image(
            self.osd_img, 0, 0, Display.LAYER_OSD0, 200)

    def _beacon_loop(self):
        import socket

        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while self.running:
            try:
                udp.sendto(
                    "K230:{}".format(RTSP_PORT),
                    ("255.255.255.255", 19124))
            except Exception:
                pass
            time.sleep(2)
        try:
            udp.close()
        except Exception:
            pass

    def _rtsp_loop(self):
        stream_data = StreamData()
        try:
            while self.running:
                os.exitpoint()
                self.encoder.GetStream(self.venc_channel, stream_data)
                for i in range(stream_data.pack_cnt):
                    data = bytes(uctypes.bytearray_at(
                        stream_data.data[i],
                        stream_data.data_size[i]))
                    self.server.rtspserver_sendvideodata(
                        RTSP_SESSION,
                        data,
                        stream_data.data_size[i],
                        1000)
                self.encoder.ReleaseStream(
                    self.venc_channel, stream_data)
        except Exception as e:
            if self.running:
                print("[RTSP] stream error:", e)

    def stop(self):
        self.running = False
        time.sleep_ms(50)
        try:
            self.server.rtspserver_stop()
            self.server.rtspserver_deinit()
        except Exception as e:
            print("[RTSP] cleanup:", e)
        try:
            self.sensor.stop()
        except Exception:
            pass
        try:
            self.encoder.Stop(self.venc_channel)
            self.encoder.Destroy(self.venc_channel)
        except Exception:
            pass
        if self.link is not None:
            try:
                del self.link
            except Exception:
                pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


def restore_uart3_pins():
    """Camera/media initialization can overwrite FPIOA; restore UART3."""
    from machine import FPIOA

    fpioa = FPIOA()
    fpioa.set_function(UART_RX_PIN, FPIOA.UART3_RXD)
    fpioa.set_function(UART_TX_PIN, FPIOA.UART3_TXD)
    print("[steel_ball] UART3 pins restored after media init")


def main():
    vision = None
    detector = None
    uart = None
    try:
        # Configure UART first, matching the proven main_256.py sequence.
        uart = init_uart()
        vision = SensorWifi()
        restore_uart3_pins()

        detector = SteelBallDetector(
            KMODEL_PATH,
            model_input_size=MODEL_INPUT_SIZE,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            nms_threshold=NMS_THRESHOLD,
            rgb888p_size=RGB888P_SIZE,
            display_size=vision.display_size,
            debug_mode=0)
        detector.config_preprocess()
        vision.start()

        fps = 0
        fps_count = 0
        fps_timer = time.ticks_ms()
        frame_index = 0
        print("[steel_ball+WiFi] ready")

        while True:
            os.exitpoint()
            img = vision.get_frame()
            if img is None:
                continue

            dets = detector.run(img)
            detector.draw_result(vision, dets)

            fps_count += 1
            elapsed = time.ticks_diff(time.ticks_ms(), fps_timer)
            if elapsed >= 1000:
                fps = fps_count
                fps_count = 0
                fps_timer = time.ticks_ms()

            best_center, best_score, count = overlay_osd(
                dets, vision, frame_index, fps)
            if (frame_index % UART_SEND_INTERVAL) == 0:
                msg = build_uart_msg(
                    best_center, best_score, count, _get_center())
                send_uart(uart, msg)

            vision.show_image()
            frame_index += 1
            gc.collect()

    except KeyboardInterrupt:
        print("[steel_ball+WiFi] stopped by user")
    except Exception as e:
        import sys
        print("[steel_ball+WiFi] error:", e)
        sys.print_exception(e)
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except Exception as e:
                print("[steel_ball] detector cleanup:", e)
        if vision is not None:
            vision.stop()
        gc.collect()
        print("[steel_ball+WiFi] shutdown")


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
