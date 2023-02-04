import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
import time
import numpy as np
import threading
from car import stop_car


model = 'efficientdet_lite0.tflite'
width = 480
height = 360
threads = 3
enableTPU = False

labels = ['stop sign', 'traffic cone', 'traffic light']
base_options = core.BaseOptions(
      file_name=model, use_coral=enableTPU, num_threads=threads)
detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.4, category_name_allowlist=labels)
options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)


def start_camera():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    
    return cap

def get_detect(cap):
    success, image = cap.read()
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    input_tensor = vision.TensorImage.create_from_array(rgb_image)
    detection_result = detector.detect(input_tensor)
    detects = [d.categories[0].category_name for d in detection_result.detections]
    
    return detects

def parse_detects(detects):
    for detect in detects:
        if detect == 'stop sign':
            stop_car()
            print('stopping')
            time.sleep(1)
        elif detect == 'traffic cone':
            return True
        else:
            None #implement traffic light
    return False
            
