#!/usr/bin/env python
import os.path
import time
from flask import Flask
from flask import request

IMAGE_FOLDER = "/opt/ipc/image"
RESULT_FOLDER = "/opt/ipc/result"
DELAY = 0.01

app = Flask(__name__)

@app.route('/', methods=['POST'])
def receive_image():
    content_type = request.headers.get("Content-Type")
    if content_type != "image/jpeg":
        return "Content must be jpg image", 406

    data = request.data
    # Assumes no more than 1 request per second
    timestamp = time.time()
    image_file_name = "{}/{}.jpg".format(IMAGE_FOLDER, timestamp)
    result_file_name = "{}/{}.jpg".format(RESULT_FOLDER, timestamp)

    with open(image_file_name, 'w') as f:
        f.write(data)

    while not os.path.isfile(result_file_name):
        time.sleep(DELAY)
    with open(result_file_name) as f:
        contents = ""
        while not contents.endswith("done"):
            time.sleep(DELAY)
            f.seek(0)
            contents = f.read()
    contents = contents[:-5]
    return contents, 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
