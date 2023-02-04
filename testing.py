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

#sweep = [114.47, -2, -2, 35.48, 35.72, 33.15, 33.29, 32.97, 34.89, 36.6, 118.84, 64.05, 64.21]

start = (0,14)
goal = (15,14)
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

print(mapping)

path, cost_so_far = a_star(start, goal, mapping)
route = make_route(path, start, goal)
route.reverse()
route.append(goal)

instructions = generate_instructions(route, start)
print(instructions)
moves = generate_movements(instructions)

print(moves)

stop_car()
