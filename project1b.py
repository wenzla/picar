import time
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
from navigation import make_scan_list, make_safety_zone, a_star, make_route, generate_instructions, generate_movements
import utils
import numpy as np
from car import us_sweep, startup, stop_car


startup()

sweep = us_sweep()
print(sweep)

start = (0,14)
goal = (20,14)
mapping = np.zeros([30,30], dtype='int')
mapping[start] = 9
mapping[goal] = 9

angles = np.deg2rad(np.arange(-60,70,10)) #in radians
xvals = (np.round((np.sin(angles) * sweep)/5, 0)+14).astype(int)
yvals = ((np.cos(angles) * sweep)//5).astype(int)
scan_list = make_scan_list(list(zip(yvals, xvals))) 
for pair in scan_list:
    if pair[0] != -1:
        mapping[pair[0]][pair[1]] = 1
        mapping = make_safety_zone(pair[0],pair[1],mapping)

path, cost_so_far = a_star(start, goal, mapping)
route = make_route(path, start, goal)
route.reverse()
route.append(goal)

instructions = generate_instructions(route, start)
generate_movements(instructions)

#left_turn()
#right_turn()
#backward_step()

stop_car()