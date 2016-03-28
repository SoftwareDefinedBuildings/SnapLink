#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import cv2
import requests

SERVER_ADDR = "http://castle.cs.berkeley.edu:50021"

def test_file(filename):
    print filename
    obj_name = os.path.basename(filename).split(".")[0]
    img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

    rows, cols = img.shape
    if rows/cols == 480/640:
        img = cv2.transpose(img)
        img = cv2.flip(img, 1) # flip along y axis
    elif cols/rows == 480/640:
        pass
    else:
        print "wrong image aspect ratio"
        return False

    img = cv2.resize(img, (480, 640)) # scale
    img = cv2.flip(img, 0) # flip along x axis TODO: why flip?
    
    files = {'file': (filename, img.tostring())}
    r = requests.post(SERVER_ADDR, files=files)
    if r.text != obj_name:
        print "test failed. response =", r.text, "obj =", obj_name
        return False
    print "test passed. response =", r.text, "obj =", obj_name
    print ""
    return True


def test_dir(directory):
    for filename in glob.glob(os.path.join(directory, "*.jpg")):
        if not test_file(filename):
            return False
    print "all tests passed"
    return True

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
