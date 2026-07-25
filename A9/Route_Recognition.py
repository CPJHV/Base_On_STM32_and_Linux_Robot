import cv2
import numpy as np
import math

img = cv2.imread("test.jpg")
if img is None:
    print("图片读取失败！")
    exit()

gray = None
hsv = None
vihicle_width=100 #车宽度
h,w=img.shape[:2] #获取图像高度和宽度
centerx=w//2
pointA=(int(centerx-vihicle_width/2),int(h))
pointB=(int(centerx+vihicle_width/2),int(h))
router_wight=200
router_offect=10#车与路的边缘的最小差距

def nothing(x):
    pass
def process_image(thresh_val,thresh_val2,ydraw,number):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("窗口名称", gray)
 
    ret, binary = cv2.threshold(gray, 215, 255, cv2.THRESH_BINARY)
    cv2.imshow("窗口名称2",binary )
    kernel = np.ones((5, 5), np.uint8)
    result = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    cv2.imshow("窗口名称3",result )

    open_img = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    contours, hierarchy = cv2.findContours(open_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    canvas = np.zeros_like(img)
    min_area = thresh_val2   # 小于这个面积的直接删掉
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:
        # 每隔10个点取一个（超少量点）
            sparse_points = cnt[::number]  
        
        # 只画点，不连线！
            for point in cnt[::number]:
               x, y = point[0]
               cv2.circle(canvas, (x, y), 3, (0,255,0), -1)
    
    # 4. 画红点
    cv2.circle(canvas, pointA, 8, (0, 0, 255), -1)  # 红、实心
    cv2.circle(canvas, pointB, 8, (0, 0, 255), -1)

# 5. 标注文字 A、B
    cv2.putText(canvas, "A", (pointA[0]-15, pointA[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    cv2.putText(canvas, "B", (pointB[0]-15, pointB[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    y = h - 1
    green_x = []
    

    for x in range(w):
        b, g, r = canvas[ydraw, x]
        # 判断绿色（轮廓颜色）
        if g==255:
            green_x.append(x)

# 开始标记
    x1=0
    if len(green_x) >= 2:
    # 有2个以上：只标第一个 + 最后一个
        x1 = green_x[0]
        x2 = green_x[-1]
        cv2.circle(canvas, (x1, ydraw), 10, (255, 0, 0), -1)  # 蓝点
        cv2.circle(canvas, (x2, ydraw), 10, (255, 0, 0), -1)
        router_wight=x2-x1
        router_wight//=2
    elif len(green_x) == 1:
    # 只有1个：只标这一个
        x1 = green_x[0]
        cv2.circle(canvas, (x1, ydraw), 10, (255, 0, 0), -1)
    
     #计算向量
    xm=x1-pointA[0]  #水平向量
    #两个点实际距离+要移动的位移=需要保持的距离
    move_offect=(router_offect-xm)
    ym=1
    
    length = math.sqrt(move_offect*move_offect + ym*ym)
    # 弧度
    rad = math.atan2(1, move_offect)
    angle = math.degrees(rad)
    angle_text = f"{angle:.1f}°"
    cv2.arrowedLine(canvas, (centerx,h), (centerx+move_offect,200), (0,255,255), 2, tipLength=0.2)
    cv2.putText(
    canvas,        # 画在哪张图
    angle_text,    # 要显示的文字（角度）
    (50, 50),  # 文字位置坐标
    cv2.FONT_HERSHEY_SIMPLEX,  # 字体
    0.8,           # 字体大小
    (0, 255, 255), # 文字颜色 BGR 黄色
    2              # 线条粗细
    )
    cv2.imshow("最终保留大轮廓", canvas)

if __name__ == "__main__":
    
    cv2.namedWindow("TrackBar", cv2.WINDOW_NORMAL)
    # 2. 创建滑块：名字、窗口、默认值、最大值、回调
    cv2.createTrackbar("Thresh", "TrackBar", 127, 255, nothing)
    cv2.createTrackbar("Thresh2", "TrackBar", 127, 255, nothing)
    cv2.createTrackbar("Thresh3", "TrackBar", 0, 391, nothing)
    cv2.createTrackbar("Thresh4", "TrackBar", 20, 255, nothing)
    while True:
        # 3. 实时读滑块值
        thresh_val = cv2.getTrackbarPos("Thresh", "TrackBar")
        thresh_val2 = cv2.getTrackbarPos("Thresh2", "TrackBar")
        thresh_val3 = cv2.getTrackbarPos("Thresh3", "TrackBar")
        thresh_val4 = cv2.getTrackbarPos("Thresh4", "TrackBar")
        process_image(thresh_val,thresh_val2, thresh_val3,thresh_val4)
        # 按ESC退出
        if cv2.waitKey(30) & 0xFF == 27:
            break

    cv2.destroyAllWindows()