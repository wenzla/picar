import time
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
import numpy as np
from car import us_sweep, startup, stop_car


startup()

print(us_sweep())
#left_turn()
#right_turn()
#backward_step()

stop_car()