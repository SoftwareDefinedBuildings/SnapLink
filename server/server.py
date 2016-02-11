#!/usr/bin/env python
import os.path
import time
from flask import Flask
from flask import request

IMAGE_FOLDER = "/root/data/ipc/image"
RESULT_FOLDER = "/root/data/ipc/result"
PORT = 8080
DELAY = 0.01
TIMEOUT = 5

app = Flask(__name__)

@app.route('/', methods=['POST'])
def receive_image():
    content_type = request.headers.get("Content-Type")
    if content_type != "image/jpeg":
        return "Content must be jpg image", 406

    data = request.data
    # image file name has to increase for RTABMap to read
    timestamp = time.time()
    print "timestamp", timestamp
    image_file_name = "{}/{}.jpg".format(IMAGE_FOLDER, timestamp)
    result_file_name = "{}/{}.jpg.txt".format(RESULT_FOLDER, timestamp)

    with open(image_file_name, 'w') as f:
        f.write(data)

    timer = 0
    while not os.path.isfile(result_file_name):
        time.sleep(DELAY)
        timer += DELAY
        if (timer >= TIMEOUT):
            print "time out"
            return "None", 200
    with open(result_file_name) as f:
        contents = f.read()
        print contents
    return contents, 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT, debug=True, threaded=True)
