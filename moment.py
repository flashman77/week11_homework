import cv2 as cv
import numpy as np
import threading, time
import SDcar 

def func_thread():
    i = 0

def key_cmd(which_key):
    print('which_key', which_key)
    is_exit = False
    global enable_linetracing # if assignment express global keyword
    if which_key & 0xFF == 184:
        print('up')
        car.motor_go(100)
    elif which_key & 0xFF == 178:
        print('down')
        car.motor_back(100)
    elif which_key & 0xFF == 180:
        print('left')     
        car.motor_left(100)   
    elif which_key & 0xFF == 182:
        print('right')   
        car.motor_right(100)            
    elif which_key & 0xFF == 181:
        car.motor_stop()
        print('stop')   
    elif which_key & 0xFF == ord('q'):  
        car.motor_stop()
        print('exit')        
        is_exit = True
    elif which_key & 0xFF == ord('e'):
        enable_linetracing = True
        print('enable_linetracing: ', enable_linetracing)
    elif which_key & 0xFF == ord('w'):
        enable_linetracing = False
        car.motor_stop()
        print('enable_linetracing 2: ', enable_linetracing)
    return is_exit  

def show_grid(img):
    h, _, _ = img.shape
    for x in v_x_grid:
        cv.line(img, (x, 0), (x, h), (0, 255, 0), 1, cv.LINE_4)

def line_tracing(cx):
    global moment
    global v_x
    tolerance = 0.01
    diff = 0
    kp = 0.5  # P 제어 계수 (적절히 조정 필요)
    control_signal = 0 # P 제어를 위한 제어 신호

    if moment[0] != 0 and moment[1] != 0 and moment[2] != 0:
        avg_m = np.mean(moment)
        diff = np.abs(avg_m - cx) / v_x

        # P 제어 신호 계산 (음수 또는 양수로 방향 조정)
        control_signal = np.abs(cx - avg_m ) 
        print(control_signal)

    print('diff ={:.4f}'.format(diff))

    if diff <= tolerance:

        moment[0] = moment[1]
        moment[1] = moment[2]
        moment[2] = cx
        
        if v_x_grid[3] <= cx < v_x_grid[5]:
            car.motor_go(70)
            print('go')
        elif ((v_x_grid[3] > cx) ):
            car.motor_left(50)
            print('turn left')
        elif ((v_x_grid[5] < cx) ):
            car.motor_right(50)
            print('turn right')
        
    else:
        car.motor_go(70)
        print('go')
        moment = [0, 0, 0]

def detect_maskY_BGR(frame):
    B = frame[:, :, 0]
    G = frame[:, :, 1]
    R = frame[:, :, 2]
    Y = np.zeros_like(G, np.uint8)
    # need to tune params
    Y = G*0.5 + R*0.5 - B*0.9 # when calculating variable turn to float64
    Y = Y.astype(np.uint8)
    Y = cv.GaussianBlur(Y, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    _, mask_Y = cv.threshold(Y, 100, 255, cv.THRESH_BINARY)
    return mask_Y

def detect_maskY_HSV(frame):
    crop_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    crop_hsv = cv.GaussianBlur(crop_hsv, (5,5), cv.BORDER_DEFAULT)
    # need to tune params
    mask_Y = cv.inRange(crop_hsv, (25, 50, 100), (35, 255, 255))
    return mask_Y

def cal_moment(img, i, j):
    M = 0
    for a in range(img.shape[0]):
        for b in range(img.shape[1]):
            M += (a)**j*(b)**i*img[a,b]
    return M

def main():

    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    
    try:
        while( camera.isOpened() ):
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            cv.imshow('camera',frame)

            # image processing start here
            crop_img = frame[120:,:]
            maskY = detect_maskY_BGR(crop_img)
            cv.imshow('maskY',maskY)
            contours, _ = cv.findContours(maskY, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key = cv.contourArea)
                m = cv.moments(c)

                cx = int(m['m10']/(m['m00']+0.0001)) # two + epsilon
                cy = int(m['m01']/(m['m00']+0.0001))
                cv.circle(crop_img, (cx,cy), 3, (0,0,255), -1)
                cv.drawContours(crop_img, contours, -1, (0,255,0), 3)

                cv.putText(crop_img, str(cx), (10, 10), cv.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))
                if enable_linetracing == True:
                    line_tracing(cx)

            show_grid(crop_img)

            cv.imshow('crop_img', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))

            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
            if is_exit is True:
                cv.destroyAllWindows()
                break
    except Exception as e:
        print(e)
        global is_running
        is_running = False

if __name__ == '__main__':

    v_x = 640
    v_y = 480
    v_x_grid = [int(v_x*i/10) for i in range(1,10)]

    moment = np.array([0, 0, 0])

    print(v_x_grid)

    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()

    car = SDcar.Drive()
    
    is_running = True
    enable_linetracing = False
    main() 
    is_running = False
    car.clean_GPIO()
    print('end vis')