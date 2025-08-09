import time, os, sys, math
#可用
from media.sensor import * #导入sensor模块，使用摄像头相关接口
from media.display import * #导入display模块，使用display相关接口
from media.media import * #导入media模块，使用meida相关接口


from machine import UART
from machine import FPIOA

fpioa = FPIOA()

# UART1代码
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)

uart=UART(UART.UART1,9600)


#3.5寸mipi屏分辨率定义
lcd_width = 800
lcd_height = 480

'''
#2.4寸mipi屏分辨率定义
lcd_width = 640
lcd_height = 480
'''

sensor = Sensor(width=1280, height=960) #构建摄像头对象2
sensor.reset() #复位和初始化摄像头
sensor.set_framesize(width=320, height=240) #设置帧大小为LCD分辨率(800x480)，默认通道0
sensor.set_pixformat(Sensor.RGB565) #设置输出图像格式，默认通道0

Display.init(Display.ST7701, width=lcd_width, height=lcd_height, to_ide=True) #同时使用mipi屏和IDE缓冲区显示图像
#Display.init(Display.VIRT, sensor.width(), sensor.height()) #只使用IDE缓冲区显示图像

MediaManager.init() #初始化media资源管理器

sensor.run() #启动sensor

clock = time.clock()

# 颜色阈值定义（来自K230）
RED_LASER_THRESHOLD = (60, 100, 15, 127, 0, 127)
#GREEN_LASER_THRESHOLD = (70, 100, -64, -25, -32, 32)
GREEN_LASER_THRESHOLDS = [(30, 100, -64, -9, -32, 32),(70, 100, -64, -25, -32, 32)]
# 协议解析状态机变量
rx_state = 0
rx_data_len = 0
rx_buffer = []
rx_checksum = 0
trigger_rect = 0
rect_coords = []



def sort_corners_clockwise(points):
    """将四个点按顺时针顺序排序（左上开始）"""
    if len(points) != 4:
        return points

    # 计算中心点
    center_x = sum(p[0] for p in points) / 4
    center_y = sum(p[1] for p in points) / 4

    # 计算极角并排序
    angles = []
    for p in points:
        dx = p[0] - center_x
        dy = p[1] - center_y
        angles.append(math.atan2(dy, dx))

    # 逆时针排序后反转得到顺时针
    sorted_points = [p for _, p in sorted(zip(angles, points), key=lambda x: x[0], reverse=False)]

    # 重新排列起点为左上附近
    x_sorted = sorted(sorted_points, key=lambda p: p[0])
    left_points = x_sorted[:2]
    left_sorted = sorted(left_points, key=lambda p: p[1])

    start_index = sorted_points.index(left_sorted[0])
    return sorted_points[start_index:] + sorted_points[:start_index]

def pack_data(data, flag):
    """数据打包函数"""
    header = 0xFF
    footer = 0xFE
    buffer = bytearray([header, flag])
    for num in data:
        num = max(0, min(num, 320))
        buffer.append((num >> 8) & 0xFF)
        buffer.append(num & 0xFF)
    buffer.append(footer)
    buffer.append(footer)
    return buffer

# 时间控制参数
clock = time.clock()
last_send = time.ticks_ms()
send_red_next = True
last_laser_send = time.ticks_ms()

laser_interval = 10

while True:
    clock.tick()
    img = sensor.snapshot()
    current_time = time.ticks_ms()

    # UART协议解析状态机
    def parse_uart():
        """K230专用协议解析函数"""
        global trigger_rect
        byte = uart.read(3)
        if byte:
            if byte[0] == 0xff and byte[-1] == 0xfe:
                if trigger_rect == 0:
                    trigger_rect = 1
                    print('set Trigger_rect 1')
                if trigger_rect == 2 and len(byte) >= 3 and 0 <= int(byte[1]) <= 3:
                     # 发送坐标时补充flag参数
                        uart.write(pack_data(rect_coords[int(byte[1])], 0x01))
                        print('send:', rect_coords[int(byte[1])])

    # 矩形检测逻辑
    parse_uart()

    if trigger_rect == 1:
        rects = list(img.find_rects(threshold=15000))
        rects_sorted = sorted(rects, key=lambda r: (r.rect()[2] * r.rect()[3]), reverse=True)

        if len(rects_sorted) >= 2:
            # 处理外边框
            outer = rects_sorted[0]
            outer_ordered = sort_corners_clockwise(outer.corners())

            # 处理内边框
            inner = rects_sorted[1]
            inner_ordered = sort_corners_clockwise(inner.corners())

            # 绘制外框连线和标签
            for i in range(4):
                start = outer_ordered[i]
                end = outer_ordered[(i+1)%4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(255,0,0), thickness=2)
                img.draw_string(start[0]+5, start[1]+5, f"x{i+1}", color=(255,0,0))
                img.draw_circle(start[0], start[1], 5, color=(255,0,0))

            # 绘制内框连线和标签
            for i in range(4):
                start = inner_ordered[i]
                end = inner_ordered[(i+1)%4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(0,255,0), thickness=2)
                img.draw_string(start[0]+5, start[1]+5, f"x{i+5}", color=(0,255,0))
                img.draw_circle(start[0], start[1], 5, color=(0,255,0))

            # 计算平均坐标
            all_points = outer_ordered + inner_ordered
            averaged_points = []
            for i in range(4):
                outer_p = all_points[i]
                inner_p = all_points[i+4]
                avg_x = (outer_p[0] + inner_p[0]) // 2
                avg_y = (outer_p[1] + inner_p[1]) // 2
                averaged_points.append((avg_x, avg_y))


            rect_coords = averaged_points

            # 打印坐标
            print("\n==== Coordinates ====")
            for idx, p in enumerate(all_points[:4], 1):
                print(f"Outer x{idx}: ({p[0]:.1f}, {p[1]:.1f})")
            for idx, p in enumerate(all_points[4:], 5):
                print(f"Inner x{idx}: ({p[0]:.1f}, {p[1]:.1f})")

            print("\n==== Averaged Points ====")
            for idx, (x, y) in enumerate(averaged_points, 1):
                print(f"Avg x{idx}: ({x:.1f}, {y:.1f})")

            trigger_rect = 2
            print('set Trigger_rect 2')

    # 激光检测逻辑（来自K230）
    if trigger_rect == 0 or trigger_rect == 2:
        # 红色激光检测
                red_blobs = img.find_blobs([RED_LASER_THRESHOLD],
                                         pixels_threshold=15,
                                         area_threshold=15,
                                         merge=True,
                                         margin=5)
                if red_blobs:
                    max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
                    red_coords = (max_red.cx(), max_red.cy())
                    img.draw_circle(red_coords[0], red_coords[1], 8, color=(255,0,0))
                    img.draw_string(red_coords[0]+10, red_coords[1]+10,
                                  "R:({},{})".format(*red_coords),
                                  color=(255,0,0), scale=1.5, bg_color=(0,0,0))

                # 绿色激光检测
                green_blobs = img.find_blobs(GREEN_LASER_THRESHOLDS,
                                              pixels_threshold=15,
                                              area_threshold=15,
                                              merge=True,
                                              margin=5)
                if green_blobs:
                    max_green = max(green_blobs, key=lambda b: (b.area(), -b.y()))
                    green_coords = (max_green.cx(), max_green.cy())
                    img.draw_circle(green_coords[0], green_coords[1], 8, color=(0,255,0))
                    img.draw_string(green_coords[0]+10, green_coords[1]+10,
                                  "G:({},{})".format(*green_coords),
                                  color=(0,255,0), scale=1.5, bg_color=(0,0,0))

                # 数据交替发送逻辑
                if current_time - last_laser_send >= 200:
                    if send_red_next:
                        if red_blobs and red_coords:
                            uart.write(pack_data(red_coords, 0x02))
                            send_red_next = False
                            last_laser_send = current_time
                        elif green_blobs and green_coords:
                            uart.write(pack_data(green_coords, 0x03))
                            send_red_next = True
                            last_laser_send = current_time
                    else:
                        if green_blobs and green_coords:
                            uart.write(pack_data(green_coords, 0x03))
                            send_red_next = True
                            last_laser_send = current_time
                        elif red_blobs and red_coords:
                            uart.write(pack_data(red_coords, 0x02))
                            send_red_next = False
                            last_laser_send = current_time

    Display.show_image(img,
                  x=round((lcd_width - sensor.width())/2),
                  y=round((lcd_height - sensor.height())/2))
                  # 绘制主标题（左上角）
    '''img.draw_string(5, 5,             # x,y坐标
                    "YIBAI4",          # 文本内容
                    color=(255,0,0),   # 文字颜色（红色）
                    scale=2,           # 字体放大2倍
                    mono_space=False,   # 使用比例字体
                    bg_color=(255,255,255))  # 白色背景

                  # 绘制触发状态（右上角）
    img.draw_string(280, 5,           # x,y坐标
                                 f"TR:{trigger_rect}",  # 动态文本
                                 color=(255,0,0),  # 文字颜色（红色）
                                 scale=1.5,         # 字体放大1.5倍
                                 mono_space=True,   # 使用等宽字体
                                 bg_color=(255,255,255))  # 白色背景
                                 '''
