#!/usr/bin/env python

from __future__ import division
import glob
import os
import sys
import numpy as np
import cv2
import requests
import time
from termcolor import colored
from PIL import Image, ExifTags

SERVER_ADDR = "http://kaifei.cs.berkeley.edu:50002"

RESULT_PASS = 0
RESULT_BAD_FORMAT = 1
RESULT_FAIL = 2

def test_file(filename):
    obj_name = os.path.basename(filename).split(".")[0]

    img = Image.open(filename)
    for orientation in ExifTags.TAGS.keys():
        if ExifTags.TAGS[orientation]=='Orientation':
            break
    if not img._getexif():
        print "No EXIF"
        return RESULT_BAD_FORMAT, 0
    exif = dict(img._getexif().items())

    if exif[orientation] == 3:
        img = img.rotate(180)
    elif exif[orientation] == 6:
        img = img.rotate(270)
    elif exif[orientation] == 8:
        img = img.rotate(90)

    img = img.convert('L') # convert to grayscale
    img = np.array(img) # convert from PIL image to OpenCV image
    rows, cols = img.shape
    width = 480
    height = 640
    if rows/cols == width/height:
        width, height = height, width
    elif cols/rows == width/height:
        pass
    else:
        print "wrong image aspect ratio"
        return RESULT_BAD_FORMAT, 0

    img = cv2.resize(img, (width, height)) # scale
    
    print filename, 'width:', width, 'height:', height
    files = {'file': (filename, img.tostring()), 'width': str(width), 'height': str(height), 'fx': str(562.25), 'fy': str(562.25), 'cx': str(240), 'cy': str(320)}
    t0 = time.time()
    r = requests.post(SERVER_ADDR, files=files)
    t1 = time.time()
    elapsed_time = round((t1 - t0)*1000, 2)
    if r.text != obj_name:
        text = colored("test failed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time), 'red')
        print text
        return RESULT_FAIL, elapsed_time
    else:
        print "test passed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
        return RESULT_PASS, elapsed_time


def test_dir(directory):
    pass_count = 0
    bad_format_count = 0
    fail_count = 0
    elapsed_times = []
    for filename in glob.glob(os.path.join(directory, "*.jpg")):
        result, elapsed_time = test_file(filename)
        if result == RESULT_PASS:
            pass_count += 1
        elif result == RESULT_BAD_FORMAT:
            bad_format_count += 1
        elif result == RESULT_FAIL:
            fail_count += 1
        elapsed_times.append(elapsed_time)
    elapsed_times = np.array(elapsed_times)
    print "{0}/{1} tests passed, {2} bad format, mean time {3} +/- {4}".format(pass_count, pass_count+fail_count, bad_format_count, round(np.mean(elapsed_times), 2), round(np.std(elapsed_times), 2))

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
