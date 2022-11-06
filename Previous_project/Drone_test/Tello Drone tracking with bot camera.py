from cmath import sqrt
from djitellopy import Tello
import cv2
import numpy as np
import pygame
import time
######################################################################
width = 320  # WIDTH OF THE IMAGE
height = 240  # HEIGHT OF THE IMAGE
deadZone =50
######################################################################

startCounter =0

# CONNECT TO TELLO
me = Tello()
me.connect()
me.for_back_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0



print(me.get_battery())
me.set_video_direction(1)# 1 indicates the down vision camera and 0 indicates the front vision camera
me.streamoff()
me.streamon()
######################## 

frameWidth = width
frameHeight = height
# cap = cv2.VideoCapture(1)
# cap.set(3, frameWidth)
# cap.set(4, frameHeight)
# cap.set(10,200)



global imgContour
global dir;
global fb
global lr
global ud
def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV",640,240)
cv2.createTrackbar("HUE Min","HSV",0,179,empty)
cv2.createTrackbar("HUE Max","HSV",179,179,empty)
cv2.createTrackbar("SAT Min","HSV",0,255,empty)
cv2.createTrackbar("SAT Max","HSV",255,255,empty)
cv2.createTrackbar("VALUE Min","HSV",0,255,empty)
cv2.createTrackbar("VALUE Max","HSV",255,255,empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",166,255,empty)
cv2.createTrackbar("Threshold2","Parameters",171,255,empty)
cv2.createTrackbar("Area min","Parameters",1750,30000,empty)
cv2.createTrackbar("Area max","Parameters",10000,30000,empty)

def init():
    pygame.init()
    win = pygame.display.set_mode((400,400))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get():pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))
    
    if keyInput[myKey]:
        ans = True
    
    pygame.display.update()

    return ans
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver
def platform_observer(platform_ob,cx,cy,dronestate):
    
    K1=-10
    K2=-0.07
    px=dronestate[0]+(-cx+int(frameWidth/2))/20## platform x## Here is the possible problem source
    py=dronestate[1]+(cy-int(frameHeight/2))/20## platform y## Maybe build up a coordinate based on the beginning point of the flight
    # pz=platform_state[2]## platform z
    vqx=-me.get_speed_x()#vqx
    vqy=-me.get_speed_y()#vqy
    # vqz=quad_state[2]#vqz
    platform_ob_dot=np.array([-vqx+platform_ob[2]+K1*(platform_ob[0]-px),-vqy+platform_ob[3]+K1*(platform_ob[1]-py),K2*(platform_ob[0]-px),K2*(platform_ob[1]-py)])
    return platform_ob_dot

def track_controller_new(platform_state_ob_dot,cx,cy):
    global fb,lr
    T1=1/4
    T2=1/4
    x1=np.sqrt(((int(frameHeight / 2)-cy))**2+((int(frameWidth/2)-cx))**2)/20#R
    x2=np.pi-np.arctan2(cy-int(frameHeight / 2),cx-int(frameWidth/2))#sigma
    x3=-me.get_speed_x()#vqx
    x4=-me.get_speed_y()#vqy
    d1=platform_state_ob_dot[0]#vpx
    d2=platform_state_ob_dot[1]#vpy
    if x1 < 0.1:
        u1=d1
        u2=d2
    else:
        u1=d1+T1*x1*np.cos(x2)-T1*x2/x1*np.sin(x2)
        u2=d2+T2*x1*np.sin(x2)+T2*x2/x1*np.cos(x2)
    fb=int(-1/T1*(x3-u1)/10)
    lr=int(1/T2*(x4-u2)/10)
    if fb >50:
        fb=100
    if fb<-100:
        fb=-100
    if lr>100:
        lr=100
    if lr<-100:
        lr=-100
    return fb,lr
def getContours(img,imgContour,platform_ob,dronestate):
    global dir
    global lr
    global fb
    global ud
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    lr=0;fb=0;ud=0;
    platform_state_ob_dot=np.array([0,0,0,0])
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area min", "Parameters")
        areaMax = cv2.getTrackbarPos("Area max", "Parameters")
        
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            print(area)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
            cy = int(y + (h / 2))  # CENTER Y OF THE OBJECT
            # print(cx,cy)
            platform_state_ob_dot=platform_observer(platform_ob,cx,cy,dronestate)
            fb,lr=track_controller_new(platform_state_ob_dot,cx,cy)
            # lr = int(100*(int(frameHeight / 2)-cy)/frameHeight*0.6)
            # fb = int(100*(int(frameWidth/2)-cx)/frameWidth*0.6)

            # if np.sqrt(lr**2+fb**2)<5 and area < areaMax: 
            #     ud = -20
            # if np.sqrt(lr**2+fb**2)<5 and area > areaMax: 
            #     me.land
            if (cx <int(frameWidth/2)-deadZone):
                cv2.putText(imgContour, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(0,int(frameHeight/2-deadZone)),(int(frameWidth/2)-deadZone,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                dir = 1
            elif (cx > int(frameWidth / 2) + deadZone):
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2+deadZone),int(frameHeight/2-deadZone)),(frameWidth,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                dir = 2
            elif (cy < int(frameHeight / 2) - deadZone):
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),0),(int(frameWidth/2+deadZone),int(frameHeight/2)-deadZone),(0,0,255),cv2.FILLED)
                dir = 3
            elif (cy > int(frameHeight / 2) + deadZone):
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
                cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),int(frameHeight/2)+deadZone),(int(frameWidth/2+deadZone),frameHeight),(0,0,255),cv2.FILLED)
                dir = 4
            else: dir=0

            cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),(0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,(0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)
            break
        else: 
            dir=0;
            fb=0;lr=0

    return platform_state_ob_dot
def forward_euler(state,statedot,dt):
    if np.size(state)==1:
        new_state=state*1+dt*statedot
    else:
        new_state=state*1
        for i in range(np.size(statedot)):
            new_state[i]=new_state[i]+dt*statedot[i]
    return new_state
def display(img):
    cv2.line(img,(int(frameWidth/2)-deadZone,0),(int(frameWidth/2)-deadZone,frameHeight),(255,255,0),3)
    cv2.line(img,(int(frameWidth/2)+deadZone,0),(int(frameWidth/2)+deadZone,frameHeight),(255,255,0),3)
    cv2.circle(img,(int(frameWidth/2),int(frameHeight/2)),5,(0,0,255),5)
    cv2.line(img, (0,int(frameHeight / 2) - deadZone), (frameWidth,int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)
init()
platform_ob=np.array([0,0,0,0])
platform_ob_all=platform_ob*1
dronestate=np.array([0,0])
platformstate=np.array([0,0,0,0])
while True:
    time0=time.time()
    # GET THE IMAGE FROM TELLO
    frame_read = me.get_frame_read()
    myFrame = frame_read.frame
    fsize=np.array(np.shape(myFrame))
    if fsize[0]!=240:
        myFrame=np.delete(myFrame,range(240,fsize[0]),axis=0)
    
    img = cv2.resize(myFrame, (width, height))
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")


    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    platform_ob_dot=getContours(imgDil, imgContour,platform_ob,dronestate)
    display(imgContour)

    ################# FLIGHT
    if startCounter == 0:
       me.takeoff()
       startCounter = 1
    time1=time.time()
    dt=float(time0-time1)
    platform_ob=forward_euler(platform_ob,platform_ob_dot,dt)
    dronedot=np.array([-me.get_speed_x(),-me.get_speed_y()])
    dronestate=forward_euler(dronestate,dronedot,dt)
#     #Front Camera control
#     if dir == 1:
#        me.yaw_velocity = -30
#     elif dir == 2:
#        me.yaw_velocity = 30
#     elif dir == 3:
#        me.up_down_velocity= 30
#     elif dir == 4:
#        me.up_down_velocity= -30
#     else:
#        me.left_right_velocity = 0; me.for_back_velocity = 0;me.up_down_velocity = 0; me.yaw_velocity = 0
#    # SEND VELOCITY VALUES TO TELLO
#     if me.send_rc_control:
#        me.send_rc_control(me.left_right_velocity, me.for_back_velocity, me.up_down_velocity, me.yaw_velocity)
#     print(dir)

    #Bottom Camera Control
    
    if me.send_rc_control:
       me.send_rc_control(lr, fb, ud, me.yaw_velocity)
    print(lr,fb)

    stack = stackImages(0.9, ([img, result], [imgDil, imgContour]))
    cv2.imshow('Horizontal Stacking', stack)

    if getKey("l"):
        me.land()
        break

# cap.release()
cv2.destroyAllWindows()