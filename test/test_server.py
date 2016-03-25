#!/usr/bin/env python

import glob
import os
import sys
import numpy as np
import cv2
import requests

SERVER_ADDR = "http://castle.cs.berkeley.edu:50021"

def test(path):
    for filename in glob.glob(os.path.join(path, "*.jpg")):
        print filename
        obj_name = os.path.basename(filename).split(".")[0]
        img = cv2.imread(filename, 0)
        img = cv2.flip(img, 0) # flip along x axis TODO: why flip?
        files = {'file': (filename, img.tostring())}
        r = requests.post(SERVER_ADDR, files=files)
        if r.text != obj_name:
            print "test failed. response =", r.text, "obj =", obj_name
            return
    print "all tests passed"

if __name__ == "__main__":
    if len(sys.argv) == 2:
        test(sys.argv[-1])
    else:
        print "Please provide image directory"
