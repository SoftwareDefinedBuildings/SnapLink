#!/usr/bin/env python

from __future__ import division
import glob
import os
import sys
import numpy as np
import cv2
import requests
import time
import random
import string
from PIL import Image, ExifTags
from bw2python import ponames
from bw2python.bwtypes import PayloadObject
from bw2python.client import Client


#uuid

bw_client = Client()
bw_client.setEntityFromEnviron()
bw_client.overrideAutoChainTo(True)
SERVER_ADDR = "http://localhost:8080"

RESULT_PASS = 0
RESULT_BAD_FORMAT = 1
RESULT_FAIL = 2
DEFAULT_CHANNEL = "scratch.ns/cellmate"
IDENTITY_LENGTH = 10

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

    #modification from here
    identity = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(IDENTITY_LENGTH))
    done = []
    def onMessage(bw_message): #send next picture here
        done += [1]
        assert(len(bw_message.payload_objects)==1)
        for po in bw_message.payload_objects:
            if po.type_dotted == ponames.PODFText:
                done += [po.content]
    bw_client.subscribe(DEFAULT_CHANNEL + "/" + identity, onMessage)
    contents =  jpg.tostring()
    width = str(width)
    height = str(height)
    fx = str(562.25)
    fy = str(562.25)
    cx = str(240)
    cy = str(320)
    po_header = PayloadObject(ponames.PODFText, None, "Cellmate Image")
    po_identity = PayloadObject(ponames.PODFText, None, identity)
    po_contents = PayloadObject(ponames.PODFText, None, contents)
    po_height = PayloadObject(ponames.PODFText, None, height)
    po_width = PayloadObject(ponames.PODFText, None, width)
    po_fx = PayloadObject(ponames.PODFText, None, fx)
    po_fy = PayloadObject(ponames.PODFText, None, fy)
    po_cx = PayloadObject(ponames.PODFText, None, cx)
    po_cy = PayloadObject(ponames.PODFText, None, cy)

    t0 = time.time()
    bw_client.publish(DEFAULT_CHANNEL, payload_objects=(po_header,po_identity ,po_contents,po_height,po_width,po_fx,po_fy,po_cx,po_cy))

    while len(done)==0:
        time.sleep(10)
    t1 = time.time()

    elapsed_time = round((t1 - t0)*1000, 2)
    assert(len(done) == 2)
    if done[1] != obj_name:
       text = "test failed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
       print text
       return RESULT_FAIL, elapsed_time
    else:
       print "test passed. response = {0}, obj = {1}, elapsed time = {2} milliseconds".format(r.text, obj_name, elapsed_time)
       return RESULT_PASS, elapsed_time

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
