import picar_4wd as fc
import time
from picar_4wd import Servo
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.ultrasonic import Ultrasonic 
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
import numpy as np


def get_dist_at(angle):
    servo.set_angle(angle)
    time.sleep(0.2)
    distance = us.get_distance()
    return distance

def us_sweep(step_size=10 ,max_angle=60, min_angle=-60):
    dists = []
    
    for angle_to_set in range(min_angle,max_angle+step_size,step_size): #sweeps from -60 to 60, incrementing by 10
        dist = get_dist_at(angle_to_set)
        dists.append(dist)
        print(angle_to_set, dist)

    return dists

def left_turn(): #90 degree turn
    fc.turn_left(50)
    time.sleep(1.05)
    fc.stop()
    
def right_turn(): #90 degree turn; for some reason behavior different from left turn??
    fc.turn_right(40)
    time.sleep(0.95)
    fc.stop()
    
def forward_step(step=1): #moves forward 5cm * step
    fc.forward(10)
    time.sleep((0.25)*step)
    fc.stop()
    
def backward_step(step=1): #moves backward 5cm * step
    fc.backward(10)
    time.sleep((0.25)*step)
    fc.stop()

power_val = 10
servo = Servo(PWM("P0"), offset=0)
servo.set_angle(0)
time.sleep(0.2)
us = Ultrasonic(Pin('D8'), Pin('D9'))
angle_distance = [0,0]
current_angle = -60
print(us_sweep())
#left_turn()
#right_turn()
#backward_step()

#[26.9, 26.93, 27.26, 26.88, 27.72, 29.26, 42.37, 41.93, 42.47, 41.78, 42.67, 43.66, -2]


servo.set_angle(0)
fc.stop()