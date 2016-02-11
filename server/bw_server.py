#!/usr/bin/env python

import os.path
import time
from flask import Flask
from flask import request
from bw2python.bwtypes import PayloadObject
from bw2python.client import Client


PORT = 8081
KEY_FILE = "kaifei.bwkey"


def on_set_entity_response(response):
    if response.status != "okay":
        print response.reason

def on_publish_response(response):
    if response.status != "okay":
        print response.reason


app = Flask(__name__)

bw_client = Client('localhost', 28589)
bw_client.connect()
bw_client.setEntityFromFile(KEY_FILE, on_set_entity_response)

@app.route('/<device>/<value>', methods=['GET'])
def handler(device, value):
    uri = "sdb.bw2.io/demo/soda/410/plugstrip/" + device + "/binary/ctl/state"
    po = PayloadObject((1, 0, 1, 0), None, "\x00" if value == "0" else "\x01")
    bw_client.publish(uri, on_publish_response, payload_objects=(po,), elaborate_pac="full", auto_chain=True)
    return "okay", 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=PORT, debug=True)
