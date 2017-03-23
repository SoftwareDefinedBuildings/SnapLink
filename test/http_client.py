#!/usr/bin/env python

from __future__ import division
import glob
import os
import sys
import numpy as np
import cv2
import requests
import time
from PIL import Image, ExifTags

SERVER_ADDR = "http://localhost:8080"

RESULT_PASS = 0
RESULT_BAD_FORMAT = 1
RESULT_FAIL = 2

def test_file(filename):
    obj_name = os.path.basename(filename).split(".")[0]

    img = Image.open(filename)
    width, height = img.size
    if height/width == 480/640:
        width, height = 640, 480
    elif width/height == 480/640:
        width, height = 480, 640
    else:
        print "wrong image aspect ratio"
        return RESULT_BAD_FORMAT, 0

    for orientation in ExifTags.TAGS.keys():
        if ExifTags.TAGS[orientation]=='Orientation':
            break
    if not img._getexif():
        print "No EXIF"
        return RESULT_BAD_FORMAT, 0
    exif = dict(img._getexif().items())

    if exif[orientation] == 3:
        img = img.rotate(180, expand=True)
    elif exif[orientation] == 6:
        img = img.rotate(270, expand=True)
        width, height = height, width
    elif exif[orientation] == 8:
        img = img.rotate(90, expand=True)
        width, height = height, width

    img.thumbnail((width, height), Image.ANTIALIAS)

    img = img.convert('L') # convert to grayscale

    img = np.array(img) # convert from PIL image to OpenCV image

    _, jpg = cv2.imencode('.jpg', img)

    print filename, 'width:', width, 'height:', height
    files = {'file': (filename, jpg.tostring()), 'fx': str(562.25), 'fy': str(562.25), 'cx': str(240), 'cy': str(320)}
    t0 = time.time()
    r = requests.post(SERVER_ADDR, files=files)
    t1 = time.time()
    elapsed_time = round((t1 - t0)*1000, 2)
    resultFile = open("410DemoTest.txt", "a+")
    if r.text != obj_name:
        text = "test failed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
        print text
        resultFile.write("fail: " + filename + " \n")
        resultFile.close()
        return RESULT_FAIL, elapsed_time
    else:
        print "test passed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
        resultFile.write("pass: " + filename + " \n")
        resultFile.close()
        return RESULT_PASS, elapsed_time


def test_dir(directory):
    pass_count = 0
    bad_format_count = 0
    fail_count = 0
    elapsed_times = []
    for filename in glob.glob(os.path.join(directory, "*")):
        if not filename.endswith(".jpg") and not filename.endswith(".JPG"):
            continue
        result, elapsed_time = test_file(filename)
        if result == RESULT_PASS:
            pass_count += 1
        elif result == RESULT_BAD_FORMAT:
            bad_format_count += 1
        elif result == RESULT_FAIL:
            fail_count += 1
        elapsed_times.append(elapsed_time)
    if elapsed_times:
        elapsed_times = np.array(elapsed_times)
        print "{0}/{1} tests passed, {2} bad format, mean time {3} +/- {4}".format(pass_count, pass_count+fail_count, bad_format_count, round(np.mean(elapsed_times), 2), round(np.std(elapsed_times), 2))
    else:
        print "No image found"

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
