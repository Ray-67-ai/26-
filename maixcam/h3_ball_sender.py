"""MaixCAM Pro 第三问钢球坐标发送程序。

接线：A19(UART1_TX) -> 天猛星 PA9(UART1_RX)
      A18(UART1_RX) <- 天猛星 PA8(UART1_TX)
      GND            -> 天猛星 GND

串口帧：B,序号,摄像头毫秒,钢球位置毫米,有效标志\n
首次使用必须在 MaixVision 中调整 ROI、管道左右端像素和 LAB 阈值。
"""

from maix import app, camera, display, err, image, pinmap, time, uart


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 只在水管区域寻找钢球，格式 [x, y, w, h]。
BALL_ROI = [10, 80, 300, 80]

# 画面中 -12.5 cm 和 +12.5 cm 对应的像素横坐标，安装后实测修改。
PIPE_NEGATIVE_END_PX = 20
PIPE_POSITIVE_END_PX = 300

# 钢球初始阈值（LAB）。钢球反光随照明变化很大，必须用“寻找色块”工具实测。
BALL_THRESHOLDS = [[0, 45, -25, 25, -25, 25]]
MIN_BLOB_PIXELS = 20
MIN_BLOB_AREA = 25
MIN_BALL_SIZE_PX = 4
MAX_BALL_SIZE_PX = 45


def pixel_to_mm(center_x):
    span = PIPE_POSITIVE_END_PX - PIPE_NEGATIVE_END_PX
    if span <= 0:
        return 0
    center_px = (PIPE_NEGATIVE_END_PX + PIPE_POSITIVE_END_PX) * 0.5
    return int(round((center_x - center_px) * 250.0 / span))


def choose_ball(blobs, last_center_x):
    best = None
    best_score = -1.0e9
    for blob in blobs:
        x, y, w, h = blob[0], blob[1], blob[2], blob[3]
        if w < MIN_BALL_SIZE_PX or h < MIN_BALL_SIZE_PX:
            continue
        if w > MAX_BALL_SIZE_PX or h > MAX_BALL_SIZE_PX:
            continue
        ratio = float(w) / float(h)
        if ratio < 0.50 or ratio > 2.0:
            continue
        center_x = x + w // 2
        area_score = float(w * h)
        tracking_penalty = 0.0
        if last_center_x is not None:
            tracking_penalty = abs(center_x - last_center_x) * 3.0
        score = area_score - tracking_penalty
        if score > best_score:
            best_score = score
            best = (x, y, w, h, center_x)
    return best


err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"),
                "UART1 TX pinmap failed")
err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"),
                "UART1 RX pinmap failed")
serial = uart.UART("/dev/ttyS1", 115200)

cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
disp = display.Display()
sequence = 0
last_center_x = None

while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs(BALL_THRESHOLDS,
                           roi=BALL_ROI,
                           pixels_threshold=MIN_BLOB_PIXELS,
                           area_threshold=MIN_BLOB_AREA,
                           merge=True,
                           margin=2)
    ball = choose_ball(blobs, last_center_x)
    now_ms = time.ticks_ms()

    if ball is None:
        valid = 0
        position_mm = 0
        last_center_x = None
    else:
        x, y, w, h, last_center_x = ball
        position_mm = pixel_to_mm(last_center_x)
        valid = 1
        img.draw_rect(x, y, w, h, image.COLOR_GREEN)

    img.draw_rect(BALL_ROI[0], BALL_ROI[1], BALL_ROI[2], BALL_ROI[3],
                  image.COLOR_BLUE)
    serial.write_str("B,{},{},{},{}\n".format(
        sequence, now_ms & 0xFFFFFFFF, position_mm, valid))
    sequence = (sequence + 1) & 0xFFFFFFFF
    disp.show(img)

