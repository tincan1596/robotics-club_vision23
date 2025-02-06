import gym
import os
import cv2 as cv
import time as t
import pybullet as p
import config_v1
import vision_v1
import numpy as np

os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('vision-v1', 
    car_location=config_v1.CAR_LOCATION,
    balls_location=config_v1.BALLS_LOCATION,
    humanoids_location=config_v1.HUMANOIDS_LOCATION,
    visual_cam_settings=config_v1.VISUAL_CAM_SETTINGS
)
t.sleep(1.5)
#lower_red = np.array([0, 50, 50])
#upper_red = np.array([10, 255, 255])
lower_green = np.array([50, 100, 100])
upper_green = np.array([70, 255, 255])
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])
lower_yellow = np.array([20,100,100])
upper_yellow = np.array([30,255,255])

def orient(key):
    while True:
        face_cam = env.get_image(cam_height=1, dims=[600,600])
        result = cv.matchTemplate(face_cam,key,cv.TM_CCORR_NORMED)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
        print(max_val , max_loc)
        if max_loc[0]>=250 and max_loc[0]<=275 and max_val>=0.74:
            env.move(vels=[[0,0],[0,0]])
            break

def Contours(i):
    img = env.get_image(cam_height = 0, dims=[600,600])
    hsv_img = cv.cvtColor(img,cv.COLOR_BGR2HSV)
    #red_mask = cv.inRange(hsv_img, lower_red, upper_red) #threshval = 35 ,but of no use :-(. 
    green_mask = cv.inRange(hsv_img, lower_green, upper_green) #threshval = 60
    blue_mask = cv.inRange(hsv_img, lower_blue, upper_blue) #threshval = 29
    yellow_mask = cv.inRange(hsv_img,lower_yellow,upper_yellow) #threshval = 116
    threshval_arr = [116 , 29 , 60]
    mask_arr = [yellow_mask , blue_mask , green_mask]
    mask=mask_arr[i]
    result = cv.bitwise_and(img, img, mask=mask)
    gray= cv.cvtColor(result,cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, threshval_arr[i], 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_KCOS)
    contours = sorted(contours, key=cv.contourArea, reverse=True)
    cv.drawContours(img, contours, 0, (0, 0, 255), 2)
    return contours , img

def goto_pole(i) :
    pole = cv.imread("pole.jpg")
    if i==0:
        env.move(vels=[[-4,4],[-4,4]])
    else:
        env.move(vels=[[4,-4],[4,-4]])
    while True:
        img = env.get_image(cam_height = 1, dims=[600,600])
        result = cv.matchTemplate(img,pole,cv.TM_CCORR_NORMED)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
        print(max_val   ,   max_loc)
        if max_loc[0]>=250 and max_loc[0]<=280 and max_val>=0.79:
                env.move(vels=[[0,0],[0,0]])
                break
    env.move(vels=[[6,6],[6,6]])
    t.sleep(4)

def FindHoid(i):
    hoid_arr = ["hoid_y.jpg","hoid_r.jpg","hoid_g.jpg","hoid_b.jpg"]
    img = cv.imread(hoid_arr[i])
    scr = env.get_image(cam_height = 1, dims=[600,600])
    result = cv.matchTemplate(scr,img,cv.TM_CCORR_NORMED)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    if max_val>=0.74 and max_loc[0]>=300:
        env.move(vels=[[1,-1],[1,-1]])
        orient(img)
    else:
        env.move(vels=[[-1,1],[-1,1]])
        orient(img)

def FindBall(i):
    ball_arr = ["y.jpg","b.jpg","g.jpg"]
    key = cv.imread(ball_arr[i],cv.IMREAD_UNCHANGED)
    face_cam = env.get_image(cam_height=0, dims=[600,600])
    result = cv.matchTemplate(face_cam,key,cv.TM_CCORR_NORMED)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    print(max_val)
    if max_val>=0.74 and max_loc[0]>=300:
        env.move(vels=[[2,-2],[2,-2]])
        orient(key)
    else:
        env.move(vels=[[-2,2],[-2,2]])
        orient(key)
    while True:
        env.open_grip()
        contours , img = Contours(i)
        area=cv.contourArea(contours[0])
        env.move(vels=[[5,5],[5,5]])
        if area >=49000 :
            env.move(vels=[[0,0],[0,0]])
            env.close_grip()
            break     

FindBall(0)
goto_pole(0)
env.move(vels=[[-2,2],[-2,2]])
t.sleep(5.5)
env.move(vels=[[0,0],[0,0]])
FindHoid(0)
env.open_grip()
env.shoot(250)
env.close_grip()

key = cv.imread("r.jpg",cv.IMREAD_UNCHANGED)
face_cam = env.get_image(cam_height=0, dims=[600,600])
result = cv.matchTemplate(face_cam,key,cv.TM_CCORR_NORMED)
min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
if max_val>=0.81 and max_loc[0]>=300:
    env.move(vels=[[2,-2],[2,-2]])
    orient(key)
else:
    env.move(vels=[[-1,1],[-1,1]])
    orient(key)
env.open_grip()
env.move(vels=[[4,4],[4,4]])
t.sleep(6.7)
env.move(vels=[[0,0],[0,0]])
env.close_grip()
goto_pole(0)
env.move(vels=[[2,2],[2,2]])
t.sleep(1)
env.move(vels=[[-3,3],[-3,3]])
t.sleep(4)
env.move(vels=[[0,0],[0,0]])
FindHoid(1)
env.open_grip()
env.shoot(250)

env.move(vels=[[-2,2],[-2,2]])
t.sleep(2)
env.move(vels=[[4,4],[4,4]])
t.sleep(3.2)
env.move(vels=[[0,0],[0,0]])
t.sleep(0.5)
env.move(vels=[[2,-2],[2,-2]])
t.sleep(1.7)
env.move(vels=[[4,4],[4,4]])
t.sleep(4.2)
env.move(vels=[[0,0],[0,0]])
t.sleep(0.5)
env.move(vels=[[2,-2],[2,-2]])
t.sleep(3)
env.move(vels=[[0,0],[0,0,]])
FindBall(2)
goto_pole(1)
env.move(vels=[[2,2],[2,2]])
t.sleep(2)
env.move(vels=[[2,-2],[2,-2]])
t.sleep(5.6)
env.move(vels=[[0,0],[0,0]])
FindHoid(2)
env.open_grip()
env.shoot(250)
env.close_grip()

env.move(vels=[[4,-4],[4,-4]])
t.sleep(3)
env.move(vels=[[0,0],[0,0]])
FindBall(1)
env.move(vels=[[-4,4],[-4,4]])
t.sleep(4)
env.move(vels=[[0,0],[0,0]])
goto_pole(0)
env.move(vels=[[4,-4],[4,-4]])
t.sleep(2.7)
FindHoid(3)
env.open_grip()
env.shoot(250)
env.close_grip()
t.sleep(10)
