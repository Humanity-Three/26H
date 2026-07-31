"""
wt5_ap.py — K230 RTSP 推流 + blob 检测 (AP 热点模式)
====================================================
K230 自己开 WiFi 热点，PC 直连，无需路由器/手机。
用法：
  1. 在 K230 上运行此文件
  2. PC 搜索 WiFi "K230-RTSP"，密码 "12345678"，连上
  3. PC 运行 pc_rtsp.py 自动发现并拉流

注意：依赖庐山派的 WiFi 芯片支持 AP 模式。
      如果运行时报 wlan 相关错误，说明固件不支持 AP 模式。
"""

import os, time, uctypes, _thread
from media.vencoder import *
from media.sensor import *
from media.display import *
from media.media import *
import multimedia as mm

W, H = 640, 480

# ========== WiFi 热点配置 ==========
AP_SSID = 'K230-RTSP'       # PC 搜索到的 WiFi 名称
AP_PASS = '12345678'         # WiFi 密码


class Rtsp:
    def __init__(self):
        import socket, network

        # --- 创建 WiFi 热点 ---
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(ssid=AP_SSID, key=AP_PASS)
        # 等热点就绪（获取到 IP）
        while ap.ifconfig()[0] == '0.0.0.0':
            os.exitpoint()
            time.sleep(0.5)
        print("[WiFi] 热点已开启: {}  IP: {}".format(AP_SSID, ap.ifconfig()[0]))

        # --- 多媒体初始化 ---
        MediaManager.init()
        self.sensor = Sensor(id=2, width=W, height=H, fps=90)
        self.sensor.reset()
        self.sensor.set_framesize(width=W, height=H, alignment=12, chn=CAM_CHN_ID_0)
        self.sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        self.sensor.set_framesize(width=W, height=H, chn=CAM_CHN_ID_1)
        self.sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
        self.sensor.set_framesize(width=W, height=H, chn=CAM_CHN_ID_2)
        self.sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_2)

        # --- H.264 编码器 ---
        self.venc_chn = VENC_CHN_ID_0
        self.enc = Encoder()
        self.enc.SetOutBufs(self.venc_chn, 8, W, H)
        self.link = MediaManager.link(
            self.sensor.bind_info()['src'],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn))
        attr = ChnAttrStr(Encoder.PAYLOAD_TYPE_H264, Encoder.H264_PROFILE_BASELINE, W, H,
                          bit_rate=1500, gopLen=30,
                          src_frame_rate=30, dst_frame_rate=30)
        self.enc.Create(self.venc_chn, attr)

        # --- OSD 覆层（用于画检测框）---
        self.ovl = image.Image(ALIGN_UP(W, 16), H, image.ARGB8888)
        self.ovl.clear()
        Display.init(Display.ST7701, width=800, height=480, to_ide=False)
        Display.bind_layer(**self.sensor.bind_info(chn=CAM_CHN_ID_1), layer=Display.LAYER_VIDEO1)

        # --- RTSP 服务 ---
        self.srv = mm.rtsp_server()
        self.srv.rtspserver_init(8554)
        self.srv.rtspserver_createsession("k230", mm.multi_media_type.media_h264, False)
        print("[RTSP] rtsp://{}:8554/k230".format(ap.ifconfig()[0]))

        self._ap = ap   # 保存引用，后面打印 IP 用

    def start(self):
        self.srv.rtspserver_start()
        self.enc.Start(self.venc_chn)
        self.sensor.run()
        self._running = True
        _thread.start_new_thread(self._rtsp_loop, ())
        _thread.start_new_thread(self._beacon, ())

    def _beacon(self):
        """UDP 广播，让 PC 自动发现 K230"""
        import socket
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while self._running:
            try:
                udp.sendto("K230:8554", ('255.255.255.255', 19124))
            except:
                pass
            time.sleep(2)
        udp.close()

    def _rtsp_loop(self):
        """从编码器取 H.264 数据，塞给 RTSP server 发送"""
        sd = StreamData()
        try:
            while self._running:
                os.exitpoint()
                self.enc.GetStream(self.venc_chn, sd)
                for i in range(sd.pack_cnt):
                    d = bytes(uctypes.bytearray_at(sd.data[i], sd.data_size[i]))
                    self.srv.rtspserver_sendvideodata("k230", d, sd.data_size[i], 1000)
                self.enc.ReleaseStream(self.venc_chn, sd)
        except Exception as e:
            print("[RTSP]", e)

    def stop(self):
        self._running = False
        self.srv.rtspserver_stop()
        self.srv.rtspserver_deinit()
        self.sensor.stop()
        del self.link
        self.enc.Stop(self.venc_chn)
        self.enc.Destroy(self.venc_chn)
        Display.deinit()
        MediaManager.deinit()


def main():
    r = Rtsp()
    r.start()
    shape = [H, W]
    import cv_lite, ulab.numpy as np
    last = time.ticks_ms(); fps_n = 0

    try:
        while True:
            os.exitpoint()
            t0 = time.ticks_us()
            img = r.sensor.snapshot(chn=CAM_CHN_ID_2)
            if img is None:
                continue
            blob = cv_lite.grayscale_find_blobs(shape, img.to_numpy_ref(), 200, 255, 50, 1)
            r.ovl.clear()
            if len(blob) > 0:
                r.ovl.draw_rectangle(blob[0], blob[1], blob[2], blob[3],
                                     color=(0, 255, 0), thickness=1)
            Display.show_image(r.ovl, 0, 0, Display.LAYER_OSD0, 200)
            dt = time.ticks_diff(time.ticks_us(), t0)
            fps_n += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, last) >= 5000:
                print("  {}fps  work:{}us".format(fps_n * 1000 // time.ticks_diff(now, last), dt))
                fps_n = 0; last = now
    except Exception:
        pass
    finally:
        r.stop()


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
