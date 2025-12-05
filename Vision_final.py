# 这个脚本产生一个转动的值，告诉你的机器人向左或向右。
import sensor, image, time, math
from pyb import UART
from math import sqrt

# UART 3, and baudrate.
uart = UART(3, 115200)


#GRAYSCALE_THRESHOLD = [(0, 83)]
#THRESHOLD = (80, 7, -78, 6, -120, -18)
THRESHOLD = (53, 2, -50, 24, -51, -7)
# 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
# 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
# 较小的权重分配给下一个roi，以此类推。
ROIS = [
        (20, 140, 115, 20, 0.4),
        (0, 160, 100, 30, 0.2),
        (30, 120, 90, 20, 0.4),
        (200, 140, 110, 20, 0.4),
        (200, 160, 110, 30, 0.2),
        (140, 120, 150, 20, 0.4),
       ]

ROI_ARROW = (160,40,120,60)
ROI_RG = (160,20,150,80)
ROI_GATE = (180, 210, 40, 30)
#roi代表三个取样区域（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
#weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
#三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
#如上图的最下方的矩形，即(0, 100, 160, 20, 0  .7)

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0 #权值和初始化
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
#计算权值和。遍历上面的三个矩形，r[4]即每个矩形的权值。


enable_lens_corr = False # turn on for straighter lines...打开以获得更直的线条…

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...
thresholds_rg = [

      (42, 15, 31, 74, -31, 75)
    ,  # generic_red_thresholds -> index is 0 so code == (1 << 0)

    (87, 39, -100, -37, 97, -24)

    ,  # generic_green_thresholds -> index is 1 so code == (1 << 1)
]

threshold_gate = [
   (66, 14, 10, 43, -23, 66)#red
    ,
   (22, 0, -20, 15, -9, 12)#green
    ,
]
# Codes are or'ed together when "merge=True" for "find_blobs".


sensor.reset() # 初始化sensor
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565) # use grayscale.
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种

sensor.set_framesize(sensor.QVGA) # 使用QVGA的速度。
#设置图像像素大小

sensor.skip_frames(30) # 让新的设置生效。
sensor.set_auto_gain(False) # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False) # 颜色跟踪必须关闭白平衡
clock = time.clock() # 跟踪FPS帧率

def calculate_magnitude(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

while(True):
    clock.tick() # 追踪两个snapshots()之间经过的毫秒数.
    img = sensor.snapshot() # 拍一张照片并返回图像。

    centroid_sum = 0
    #利用颜色识别分别寻找三个矩形区域内的线段
    for r in ROIS:
        blobs = img.find_blobs([THRESHOLD], roi=r[0:4], merge=True)
        # r[0:4] is roi tuple.
        #找到视野中的线,merge=true,将找到的图像区域合并成一个
        if (r[0] <= 61):
            blob_cx = 0
        else:
            blob_cx = 320
        #目标区域找到直线
        if blobs:
            # 查找像素最多的blob的索引。
            largest_blob = 0
            most_pixels = 0
            for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs[i].pixels() > most_pixels:
                    most_pixels = blobs[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                    #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i

            # 在色块周围画一个矩形。
            img.draw_rectangle(blobs[largest_blob].rect(), color=(0,255,0))
            # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_cross(blobs[largest_blob].cx(),
                           blobs[largest_blob].cy())

            blob_cx = blobs[largest_blob].cx()
        #print(blob_cx)
        centroid_sum += blob_cx * r[4] # r[4] is the roi weight.
            #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    #中间公式
    #print(center_pos)
    # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
    # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
    deflection_angle = 0
    #机器人应该转的角度

    # 80是X的一半，60是Y的一半。
    # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
    # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。

    deflection_angle = -math.atan((center_pos-160)/60)
    #角度计算.80 60 分别为图像宽和高的一半，图像大小为QVGA 320x240.
    #注意计算得到的是弧度值


    deflection_angle = round(math.degrees(deflection_angle), 2)
    print(deflection_angle)
    ##将计算结果的弧度值转化为角度值
    if (deflection_angle > 9.9):
        # 现在你有一个角度来告诉你该如何转动机器人。
        # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。
        # Convert to string with one decimal place
        formatted_angle = "{:.1f}".format(deflection_angle)

        # Write to UART
        uart.write("aa" +formatted_angle)
        if (uart.any()):
            print(uart.read())
    elif (deflection_angle <= 9.9 and deflection_angle >= 0):
        formatted_angle = "{:.1f}".format(deflection_angle)

        # Write to UART
        uart.write("aaa" +formatted_angle)
        if (uart.any()):
            print(uart.read())
    elif (deflection_angle >= -9.9 and deflection_angle <= 0):
        formatted_angle = "{:.1f}".format(deflection_angle)

        # Write to UART
        uart.write("aa" +formatted_angle)
        if (uart.any()):
            print(uart.read())
    elif (deflection_angle < -9.9):
        formatted_angle = "{:.1f}".format(deflection_angle)

        # Write to UART
        uart.write("a" +formatted_angle)
        if (uart.any()):
            print(uart.read())
    #time.sleep_ms(100)
    #print("Turn Angle: %f" % deflection_angle)
    #将结果打印在terminal中

        #print(clock.fps())
    # 注意: 当连接电脑后，OpenMV会变成一半的速度。当不连接电脑，帧率会增加。
    # 打印当前的帧率。
####################################################################################################
    if enable_lens_corr:
        img.lens_corr(1.8)  # Apply lens correction for a 2.8mm lens

    selected_lines = []
    direction = "NULL"
    flag = "N"



    for l in img.find_line_segments(roi=ROI_ARROW, merge_distance=3, max_theta_diff=3):
        #print(l)
        if calculate_magnitude(l.x1(), l.y1(), l.x2(), l.y2())<17:
            if (50 <= l.theta() <= 70) or (110 <= l.theta() <= 140):
                img.draw_line(l.line(), color=(255, 0, 0))
                selected_lines.append((l.x1(), l.y1(), l.x2(), l.y2()))
                if len(selected_lines) >= 2:
                    combined_lines = []
                    line1, line2 = selected_lines[0], selected_lines[1]
                    combined_lines.append((line1[0], line1[1], line2[2], line2[3]))
                    combined_lines.append((line1[2], line1[3], line2[0], line2[1]))

                    magnitudes = [calculate_magnitude(*line) for line in combined_lines]

                    for i, mag in enumerate(magnitudes):
                        if mag < 5:
                            x1, y1 = combined_lines[i][:2]
                            other_line = combined_lines[1 - i]

                            if x1 < other_line[0] and x1 < other_line[2]:
                                direction = "Left"
                                flag = "L"
                            else:
                                direction = "Right"
                                flag = "R"
                    print(direction)
                    selected_lines = []
                    if direction != "NULL":  # Stop the loop if direction is detected
                        break
                img.draw_rectangle(ROI_ARROW, color=(255, 0, 0))
            elif (20 <= l.theta() <= 40) or (140 <= l.theta() <= 160):
                img.draw_line(l.line(), color=(255, 0, 0))
                selected_lines.append((l.x1(), l.y1(), l.x2(), l.y2()))
                if len(selected_lines) >= 2:
                    combined_lines = []
                    line1, line2 = selected_lines[0], selected_lines[1]
                    combined_lines.append((line1[0], line1[1], line2[2], line2[3]))
                    combined_lines.append((line1[2], line1[3], line2[0], line2[1]))

                    magnitudes = [calculate_magnitude(*line) for line in combined_lines]
                    direction = "NULL"
                    for i, mag in enumerate(magnitudes):
                        if mag < 7:
                            x1, y1 = combined_lines[i][:2]
                            other_line = combined_lines[1 - i]

                            if y1 < other_line[1] and y1 < other_line[3]:
                                direction = "Up"
                                flag = "U"
                    print(direction)
                    selected_lines = []
                    if direction != "NULL":  # Stop the loop if direction is detected
                        break
                img.draw_rectangle(ROI_ARROW, color=(255, 0, 0))

    uart.write("b" + flag)
    if uart.any():
        print(uart.read())
    #time.sleep_ms(100)
####################################################################################################
    img.draw_rectangle(ROI_RG, color=(0,0,255))
    blobs = img.find_blobs(
        thresholds_rg, pixels_threshold=5, area_threshold=10, merge=True, roi = ROI_RG
    )
    if blobs:
        # 查找像素最多的blob的索引。
        largest_blob = 0
        most_pixels = 0
        for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
            if blobs[i].pixels() > most_pixels:
                most_pixels = blobs[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                    #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                largest_blob = i
        blob = blobs[largest_blob]
        if blob.code() == 1:  # r code
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(blob.x() + 2, blob.y() + 2, "r")
            #uart.write("cR")
            uart.write("cR")
            print("Red")
        elif blob.code() == 2:  # g code
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_string(blob.x() + 2, blob.y() + 2, "g")
            uart.write("cG")
            print("Green")
        else:
            uart.write("cd")
    else:
        uart.write("cc")

     # Write to UART


    if (uart.any()):
        print(uart.read())
####################################################################################################
    # State tracking
    img.draw_rectangle(ROI_GATE, color=(0,255,255))
    blobs = img.find_blobs(
        threshold_gate, pixels_threshold=5, area_threshold=1000, merge=True, roi = ROI_GATE
    )
    if blobs:
        # 查找像素最多的blob的索引。
        largest_blob = 0
        most_pixels = 0
        for blob in blobs:
            if blob.pixels() > most_pixels:
                most_pixels = blob.pixels()
                largest_blob = blob
        detecting_grass = False
        print(detecting_grass)
        if largest_blob:
            if largest_blob.code() == 2:  # Assuming '2' is the code for grass
                img.draw_rectangle(largest_blob.rect())
                img.draw_cross(largest_blob.cx(), largest_blob.cy())
                img.draw_string(largest_blob.x() + 2, largest_blob.y() + 2, "grass")
                uart.write("ds")
                print("on the grass")
    else:
        uart.write("dx")
    time.sleep_ms(100)
    #print(clock.fps())
