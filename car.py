import picar_4wd as fc
import time
from picar_4wd import Servo
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.ultrasonic import Ultrasonic 

power_val = 10
servo = Servo(PWM("P0"), offset=0)
us = Ultrasonic(Pin('D8'), Pin('D9'))
angle_distance = [0,0]
current_angle = -60

def get_dist_at(angle):
    """Gets distance from ultrasonic sensor

    Args:
        angle (int): angle to set servo motor

    Returns:
        float: distance in centimeters
    """
    servo.set_angle(angle)
    time.sleep(0.2)
    distance = us.get_distance()
    return distance

def us_sweep(step_size=10 ,max_angle=60, min_angle=-60):
    """Ultrasonic sweep of the environment

    Args:
        step_size (int, optional): change in degrees of the sweep. Defaults to 10.
        max_angle (int, optional): _description_. Defaults to 60.
        min_angle (int, optional): _description_. Defaults to -60.

    Returns:
        list: array of distances in cm
    """
    dists = []
    
    for angle_to_set in range(min_angle,max_angle+step_size,step_size): #sweeps from -60 to 60, incrementing by 10
        dist = get_dist_at(angle_to_set)
        dists.append(dist)
        # print(angle_to_set, dist)
    return dists

def left_turn(dummy=None): #90 degree turn
    fc.turn_left(50)
    time.sleep(0.9)
    fc.stop()
    
def right_turn(dummy=None): #90 degree turn; for some reason behavior different from left turn??
    # always will change, even when running twice in a row with same numbers, it has different behavior
    fc.turn_right(40)
    time.sleep(0.91)
    fc.stop()
    
def forward_step(step=1): #moves forward 5cm * step
    fc.forward(20)
    time.sleep((0.3)*step)
    fc.stop()
    
def backward_step(step=1): #moves backward 5cm * step
    fc.backward(10)
    time.sleep((0.25)*step)
    fc.stop()
    
def startup():
    servo.set_angle(0)
    time.sleep(0.2)

def stop_car():
    servo.set_angle(0)
    fc.stop()