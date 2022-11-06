import time
from djitellopy import Tello
import cv2
import pygame
import numpy as np
global img
# tello=Tello()
# tello.connect()
# print(tello.get_battery())

# ## movement test
# tello.takeoff()
# tello.send_rc_control(0,50,0,0) # forward
# sleep(2)
# tello.send_rc_control(30,0,0,0) # right
# sleep(2)
# tello.send_rc_control(0,0,0,30) # rotate cw
# sleep(2)
# tello.send_rc_control(0,0,30,0) # up
# sleep(1)
# tello.send_rc_control(0,0,0,0)
# tello.land()

# ## Image capture
# tello.streamon()
# while True:
#     img=tello.get_frame_read().frame
#     img= cv2.resize(img,(360,240))
#     cv2.imshow("Image",img)
#     cv2.waitKey(1)## 1 milisecond

## Key press module
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

# def main():
#     if getKey("LEFT"):
#         print("Left key pressed")
#     if getKey("RIGHT"):
#         print("Right key pressed")

# if __name__=='__main__':
#     init()
#     while True:
#         main()

## Keyboard Control
def getkeyboardInput():
    lr,fb,ud,yv=0,0,0,0
    speed = 30
    x=[]
    y=[]
    z=[]
    ax=[]
    ay=[]
    az=[]
    if getKey("LEFT"): lr=-speed;y=tello.get_speed_y();print({'y dir speed',y});ay=tello.get_acceleration_y()
    elif getKey("RIGHT"): lr=speed;y=tello.get_speed_y();print({'y dir speed',y});ay=tello.get_acceleration_y()

    if getKey("UP"): fb=speed;x=tello.get_speed_x();print({'x dir speed',x});ax=tello.get_acceleration_x()
    elif getKey("DOWN"): fb=-speed;x=tello.get_speed_x();print({'x dir speed',x});ax=tello.get_acceleration_x()

    if getKey("w"): ud=speed;z=tello.get_speed_z();print({'z dir speed',z});az=tello.get_acceleration_z()
    elif getKey("s"): ud=-speed;z=tello.get_speed_z();print({'z dir speed',z});az=tello.get_acceleration_z()

    if getKey("a"): yv=-speed ##ccw
    elif getKey("d"): yv=+speed ## cw

    if getKey("q"): tello.land(); time.sleep(3)
    if getKey("e"): tello.takeoff()
    if getKey("z"): 
        cv2.imwrite(f'Resouces/Images/{time.time()}.jpg',img)
        time.sleep(0.3)
    if getKey("m"):
        
        # pad = tello.get_mission_pad_id()
        # while pad==-1:
        #     tello.send_rc_control(10,10,0,30)
        #     time.sleep(0.1)
        #     pad = tello.get_mission_pad_id()
        # x=tello.get_mission_pad_distance_x()
        # y=tello.get_mission_pad_distance_y()
        # z=tello.get_mission_pad_distance_z()
        # ag=tello.get_state_field("mpry")
        # print(x,y,z,ag)
        # # # tello.go_xyz_speed_mid(x,y,50,30,pad)
        # # if np.sqrt(x**2+y**2)>3:
        # #     lr=x; fb=y
        # tello.go_xyz_speed_mid(0,0,50,10,pad)
        pad = tello.get_mission_pad_id()
        if pad==-1:
            lr=10;fb=10;yv=30
        else: 
            tello.go_xyz_speed_mid(0,0,50,10,pad)
            # ag=tello.get_state_field("mpry")
            # print(ag)
            # time.sleep(1)

    if getKey("g"):
        yaw=tello.get_yaw()    
        print(yaw)
    return [lr,fb,ud,yv,x,y,z,ax,ay,az]

tello=Tello()
tello.connect()
print(tello.get_battery())
init()
tello.streamon()
tello.enable_mission_pads()
tello.set_mission_pad_detection_direction(2)
tello.set_video_direction(1)
x=[];y=[];z=[];ax=[];ay=[];az=[]
while True:
    
    vals = getkeyboardInput()
    tello.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    if vals[4]!=[]:
        x.append(vals[4])
    if vals[5]!=[]:
        y.append(vals[5])
    if vals[6]!=[]:
        z.append(vals[6])
    if vals[7]!=[]:
        ax.append(vals[7])
    if vals[8]!=[]:
        ay.append(vals[8])
    if vals[9]!=[]:
        az.append(vals[9])
    img=tello.get_frame_read().frame
    img= cv2.resize(img,(360,240))
    cv2.imshow("Image",img)
    cv2.waitKey(1)## 1 milisecond
    if getKey("l"):
        tello.land()
        break
with open("datax.txt","w") as f:
    f.writelines(["%s\n" % item  for item in x])
f.close
with open("datay.txt","w") as f:
    f.writelines(["%s\n" % item  for item in y])
f.close
with open("dataz.txt","w") as f:
    f.writelines(["%s\n" % item  for item in z])
f.close
with open("dataax.txt","w") as f:
    f.writelines(["%s\n" % item  for item in ax])
f.close
with open("dataay.txt","w") as f:
    f.writelines(["%s\n" % item  for item in ay])
f.close
with open("dataaz.txt","w") as f:
    f.writelines(["%s\n" % item  for item in az])
f.close
cv2.destroyAllWindows()
