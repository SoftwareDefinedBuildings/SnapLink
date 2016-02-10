#!/usr/bin/env python

import uuid
from flask import Flask, request

PORT = 8080
IMAGE_FOLDER = "/root/data/test_images"

app = Flask(__name__)

@app.route('/', methods=['POST'])
def receive_image():
    content_type = request.headers.get("Content-Type")
    if not content_type.startswith("multipart/form-data"):
        return "Content must be multipart form data", 406

    image_file_name = "{}/{}.jpg".format(IMAGE_FOLDER, uuid.uuid4())
    image_file = request.files['file']
    image_file.save(image_file_name)
    return "Okay", 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT, debug=True)
