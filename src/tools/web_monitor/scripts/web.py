#!/usr/bin/python3
# coding: utf-8

from flask import Flask, url_for, render_template, request, jsonify
import requests
import time
import os

app = Flask(__name__)
root_path = os.path.dirname(os.path.realpath(__file__))

@app.route('/')
@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/snap', methods=['GET'])
def snapshot():
    address = request.args.get('address')
    url = 'http://{}:8080/snapshot?topic=/result/vision/image'.format(address)
    res = requests.get(url)
    name = "/static/images/"+time.strftime("%Y%m%d%H%M%S", time.localtime())+'.jpg'
    with open(root_path + name, 'wb') as f:
        f.write(res.content)
    return jsonify({"name": name})

if __name__ == "__main__":
    app.run()