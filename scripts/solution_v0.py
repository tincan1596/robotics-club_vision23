import gym
import os
import cv2 as cv
import time as t
import pybullet as p
import config_v0
import vision_v0

os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('vision-v0', 
    car_location=config_v0.CAR_LOCATION,
    ball_location=config_v0.BALL_LOCATION,
    humanoid_location=config_v0.HUMANOID_LOCATION,
    visual_cam_settings=config_v0.VISUAL_CAM_SETTINGS
)
env.open_grip()
env.move(vels=[[4,4],
               [4,4]])
t.sleep(3)
env.move(vels=[[0,0],
               [0,0]])
env.close_grip()
t.sleep(1)
env.move(vels=[[4,-4],
               [4,-4]])
t.sleep(2.9)
env.move(vels=[[0,0],
               [0,0]])
env.open_grip()
t.sleep(1)
env.shoot(300)

t.sleep(5)

