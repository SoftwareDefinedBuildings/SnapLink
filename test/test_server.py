#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import cv2
import requests
import time

SERVER_ADDR = "http://kaifei.cs.berkeley.edu:50002"

def test_file(filename):
    print filename
    obj_name = os.path.basename(filename).split(".")[0]
    img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

    rows, cols = img.shape
    width = 480
    height = 640
    if rows/cols == width/height:
        img = cv2.transpose(img)
        img = cv2.flip(img, 1) # flip along y axis
    elif cols/rows == width/height:
        pass
    else:
        print "wrong image aspect ratio"
        return False

    img = cv2.resize(img, (width, height)) # scale
    
    files = {'file': (filename, img.tostring()), 'width': str(width), 'height': str(height), 'fx': str(562.25), 'fy': str(562.25), 'cx': str(240), 'cy': str(320)}
    t0 = time.time()
    r = requests.post(SERVER_ADDR, files=files)
    t1 = time.time()
    elapsed_time = round((t1 - t0)*1000, 2)
    if r.text != obj_name:
        print "test failed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
        return False, elapsed_time
    else:
        print "test passed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
        return True, elapsed_time


def test_dir(directory):
    pass_count = 0
    total_count = 0
    elapsed_times = []
    for filename in glob.glob(os.path.join(directory, "*.jpg")):
        total_count += 1
        passed, elapsed_time = test_file(filename)
        if passed:
            pass_count += 1
        elapsed_times.append(elapsed_time)
    elapsed_times = np.array(elapsed_times)
    print "{0}/{1} tests passed, mean time {2} +/- {3}".format(pass_count, total_count, round(np.mean(elapsed_times), 2), round(np.std(elapsed_times), 2))

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path = sys.argv[-1]
        if os.path.isdir(path):
            test_dir(path)
        elif os.path.isfile(path):
            test_file(path)
        else:
            print "Invalid path"
    else:
        print "Please provide image directory"
