#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import cv2
import requests
import struct

SERVER_ADDR = "http://castle.cs.berkeley.edu:50021"

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
    img = cv2.flip(img, 0) # flip along x axis TODO: why flip?
    
    files = {'file': (filename, img.tostring()), 'width': ("", struct.pack("!I", width)), 'height': ("", struct.pack("!I", height))}
    r = requests.post(SERVER_ADDR, files=files)
    if r.text != obj_name:
        print "test failed. response =", r.text, "obj =", obj_name, "\n"
        return False
    else:
        print "test passed. response =", r.text, "obj =", obj_name, "\n"
        return True


def test_dir(directory):
    pass_count = 0
    total_count = 0
    for filename in glob.glob(os.path.join(directory, "*.jpg")):
        total_count += 1
        if test_file(filename):
            pass_count += 1
    print "{0}/{1} tests passed".format(pass_count, total_count)

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
