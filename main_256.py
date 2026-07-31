"""
K230 钢球实时检测 + UART 偏移数据发送
架构：PipeLine + AIBase + AI2D + nncase_runtime + aidemo

拷贝到 SD 卡 /sdcard/ 根目录：
  /sdcard/main.py
  /sdcard/steel_ball.kmodel
  /sdcard/libs/

功能：
  1. 实时检测钢球
  2. OSD：检测框、计数、最佳球十字准星、FPS
  3. UART 发送偏移给天猛星
"""

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import letterbox_pad_param
import os
import gc
import time
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo


# ======================== 配置参数 ========================

KMODEL_PATH = "/sdcard/steel_ball_v6_256.kmodel"
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [256, 256]

# --- 摄像头 ---
SENSOR_ID = 2
RGB888P_SIZE = [640, 480]
H_MIRROR = True
V_FLIP = True

# --- 显示 ---
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None

# --- 检测 ---
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3

# --- 调试 ---
PRINT_EVERY_N_FRAMES = 10

# --- UART（天猛星） ---
UART_ENABLED = True
UART_ID = 3
UART_BAUDRATE = 115200
UART_TX_PIN = 50
UART_RX_PIN = 51
UART_SEND_INTERVAL = 1
UART_DEBUG = False

# --- 基准中心点 ---
CENTER_X = 345
CENTER_Y = 192
OFFSET_SCALE = 1.0
ALIGN_DEAD_ZONE = 2


# ======================== 钢球检测器 ========================

class SteelBallDetector(AIBase):
    def __init__(self, kmodel_path, model_input_size,
                 confidence_threshold=0.5, nms_threshold=0.3,
                 rgb888p_size=None, display_size=None, debug_mode=0):
        if rgb888p_size is None:
            rgb888p_size = [224, 224]
        if display_size is None:
            display_size = [800, 480]
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.class_id = LABELS
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        self._letterbox_ratio = 1.0
        self._pad_top = 0
        self._pad_left = 0

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
            np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = (input_image_size if input_image_size
                               else self.rgb888p_size)
            top, bottom, left, right, ratio = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size)
            self._letterbox_ratio = ratio
            self._pad_top = top
            self._pad_left = left

            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])
            self.ai2d.resize(nn.interp_method.tf_bilinear,
                             nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]])

    def postprocess(self, results):
        det_res = []
        with ScopedTiming("postprocess", self.debug_mode > 0):
            data = results[0][0]
            num_preds = data.shape[1]
            ratio = self._letterbox_ratio
            top = self._pad_top
            left = self._pad_left

            for i in range(num_preds):
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
            dets = [d for d in dets
                    if self._iou(best, d) < self.nms_threshold]
        return keep

    def _iou(self, a, b):
        ax, ay, aw, ah = a[:4]
        bx, by, bw, bh = b[:4]
        ax1 = ax - aw / 2
        ay1 = ay - ah / 2
        ax2 = ax + aw / 2
        ay2 = ay + ah / 2
        bx1 = bx - bw / 2
        by1 = by - bh / 2
        bx2 = bx + bw / 2
        by2 = by + bh / 2
        ix = max(0, min(ax2, bx2) - max(ax1, bx1))
        iy = max(0, min(ay2, by2) - max(ay1, by1))
        inter = ix * iy
        if inter <= 0:
            return 0
        return inter / (max(1, aw * ah) + max(1, bw * bh) - inter)

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det in dets:
                    x, y, w, h = map(lambda v: int(round(v, 0)), det[:4])
                    x = x * self.display_size[0] // self.rgb888p_size[0]
                    y = y * self.display_size[1] // self.rgb888p_size[1]
                    w = w * self.display_size[0] // self.rgb888p_size[0]
                    h = h * self.display_size[1] // self.rgb888p_size[1]
                    rx = x - w // 2; ry = y - h // 2
                    pl.osd_img.draw_rectangle(
                        rx, ry, w, h,
                        color=(255, 0, 255, 0), thickness=2)
            else:
                pl.osd_img.clear()


# ======================== UART 通信 ========================

def _get_center():
    cx = CENTER_X if CENTER_X is not None else RGB888P_SIZE[0] // 2
    cy = CENTER_Y if CENTER_Y is not None else RGB888P_SIZE[1] // 2
    return cx, cy


def build_uart_msg(best_center, best_score, count, center_ref):
    """构造 UART 消息，修改此函数适配天猛星协议"""
    ref_cx, ref_cy = center_ref
    if best_center is None:
        return "DX:0,DY:0,A0,D0"
    dx = int(round((best_center[0] - ref_cx) * OFFSET_SCALE))
    dy = int(round((best_center[1] - ref_cy) * OFFSET_SCALE))
    aligned = 1 if (abs(dx) <= ALIGN_DEAD_ZONE and
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
        if UART_DEBUG:
            print("[UART] NOT INITIALIZED")
        return
    try:
        written = uart.write((msg + "\n").encode())
        if UART_DEBUG:
            print("[UART] wrote=%s %s" % (str(written), msg))
    except Exception as e:
        import sys
        print("[steel_ball] UART error:", e)
        sys.print_exception(e)


# ======================== OSD ========================

def overlay_osd(dets, pl, frame_index, fps):
    """绘制 OSD，返回 (best_center, best_score, count)"""
    count = 0
    best_score = -1.0
    best_center = None

    if dets:
        count = len(dets)
        for det in dets:
            x, y, w, h = det[:4]
            score = float(det[-1])
            if score > best_score:
                best_score = score
                best_center = (int(round(x)), int(round(y)))

    # 左上：计数
    pl.osd_img.draw_string_advanced(
        5, 5, 24, "steel: %d" % count, color=(255, 0, 255, 0))

    # 最佳球十字准星
    if best_center is not None:
        cx = best_center[0] * pl.display_size[0] // RGB888P_SIZE[0]
        cy = best_center[1] * pl.display_size[1] // RGB888P_SIZE[1]
        pl.osd_img.draw_cross(cx, cy, color=(255, 255, 255, 0),
                              size=14, thickness=3)
        pl.osd_img.draw_string_advanced(
            5, 34, 20,
            "%d,%d (%.2f)" % (best_center[0], best_center[1], best_score),
            color=(255, 255, 255, 0))
    else:
        pl.osd_img.draw_string_advanced(
            5, 34, 20, "NO BALL", color=(255, 255, 0, 0))

    # 右上：FPS
    if fps > 0:
        pl.osd_img.draw_string_advanced(
            pl.display_size[0] - 80, 5, 24,
            "FPS:%d" % fps, color=(255, 0, 255, 0))

    # 串口打印
    if frame_index % PRINT_EVERY_N_FRAMES == 0:
        if best_center is None:
            print("[steel_ball] frame=%d count=0 fps=%d" % (frame_index, fps))
        else:
            print("[steel_ball] frame=%d count=%d best=(%d,%d) s=%.3f fps=%d"
                  % (frame_index, count, best_center[0], best_center[1],
                     best_score, fps))

    return best_center, best_score, count


# ======================== 主程序 ========================

if __name__ == "__main__":
    pl = None; detector = None; uart = None
    try:
        # Keep the proven CAN_STM32_K230 order: configure UART3 before
        # camera/display initialization.
        uart = init_uart()

        pl = PipeLine(rgb888p_size=RGB888P_SIZE,
                      display_mode=DISPLAY_MODE,
                      display_size=DISPLAY_SIZE)
        pl.create(sensor_id=SENSOR_ID, hmirror=H_MIRROR, vflip=V_FLIP)
        display_size = pl.get_display_size()

        # PipeLine initialization can reconfigure board pin multiplexing.
        # Re-assert the proven UART3 pins without recreating the UART object.
        if uart is not None:
            from machine import FPIOA
            uart_fpioa = FPIOA()
            uart_fpioa.set_function(UART_RX_PIN, FPIOA.UART3_RXD)
            uart_fpioa.set_function(UART_TX_PIN, FPIOA.UART3_TXD)
            print("[steel_ball] UART3 pins restored after PipeLine.create")

        detector = SteelBallDetector(
            KMODEL_PATH, model_input_size=MODEL_INPUT_SIZE,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            nms_threshold=NMS_THRESHOLD,
            rgb888p_size=RGB888P_SIZE, display_size=display_size, debug_mode=0)
        detector.config_preprocess()

        fps = 0; fps_n = 0; fps_timer = time.ticks_ms()
        frame_index = 0

        print("[steel_ball] ready (imgsz=256), kmodel=%s" % KMODEL_PATH)

        while True:
            os.exitpoint()
            with ScopedTiming("total", 0):
                img = pl.get_frame()
                dets = detector.run(img)
                detector.draw_result(pl, dets)

                fps_n += 1
                elapsed = time.ticks_diff(time.ticks_ms(), fps_timer)
                if elapsed >= 1000:
                    fps = fps_n; fps_n = 0
                    fps_timer = time.ticks_ms()

                best_center, best_score, count = overlay_osd(
                    dets, pl, frame_index, fps)

                if (frame_index % UART_SEND_INTERVAL) == 0:
                    center_ref = _get_center()
                    msg = build_uart_msg(best_center, best_score, count, center_ref)
                    send_uart(uart, msg)

                pl.show_image()
                frame_index += 1
                gc.collect()

    except KeyboardInterrupt:
        print("[steel_ball] stopped by user")
    except Exception as e:
        import sys
        print("[steel_ball] error:", e)
        sys.print_exception(e)
    finally:
        if detector is not None:
            try: detector.deinit()
            except Exception as e: print("[steel_ball] cleanup:", e)
        if pl is not None:
            try: pl.destroy()
            except Exception as e: print("[steel_ball] cleanup:", e)
        gc.collect()
        print("[steel_ball] shutdown")
