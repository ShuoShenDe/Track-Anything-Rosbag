import os
from PIL import Image
from flask import Flask, request, jsonify, send_file
from start_tracking import start_tracking
import numpy as np
import json
import base64
import cv2
import datetime

app = Flask(__name__)


@app.route('/get_cvmat_by_sam', methods=['POST'])
def get_cvmat_by_sam():
    json_data = request.get_json()
    if 'cv_mat' not in json_data:
        return jsonify({'error': 'Missing required parameters cv_mat, pls check'}), 400
    if 'source' not in json_data:
        return jsonify({'error': 'Missing required parameters source, pls check'}), 400
    source = json_data.get('source', "denso_tri52_seg_pre_labeling.bag")
    topic = json_data.get('topic', None)
    cv_mat = json_data.get('cv_mat')

    # transfer json to numpy
    if json_data.get('cv_mat'):
        # decode Base64 data
        decoded_image_data = base64.b64decode(json_data.get('cv_mat'))
        # transfer decoded_image_data to numpy
        image_array = np.frombuffer(decoded_image_data, np.uint8)
        cv_mat = cv2.imdecode(image_array, cv2.IMREAD_GRAYSCALE)
        print("get cv_mat from client is {}".format(cv_mat))

    tracking_frame_start = json_data.get('tracking_frame_start', "0")
    tracking_frame_end = json_data.get('tracking_frame_end', "10")
    print("cv_mat is {}".format(cv_mat))
    print("tracking_frame_start is {} tracking_frame_end is {}".format(int(tracking_frame_start) - 1,
                                                                       int(tracking_frame_end) - 1))
    # cv_mat[cv_mat == 1] = 100
    # cv2.imwrite(os.path.splitext(os.path.basename(source))[0] + '_.png', cv_mat)

    current_time = datetime.datetime.now()
    formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
    print("source:{} start tracking time is: {}".format(source, formatted_time))
    source_path = os.path.join("/home/ubuntu/Documents/EFS/Personal/ShuoShen/Track-Anything-Rosbag/test_sample", source)

    masks, logits, painted_images = start_tracking(source_path, topic, cv_mat, int(tracking_frame_start) - 1,
                                                   int(tracking_frame_end) - 1)

    current_time = datetime.datetime.now()
    formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
    print("source:{} finish tracking time is: {}".format(source, formatted_time))

    json_data = {
        "result": []
    }
    frame_id = int(tracking_frame_start)
    for frame_array in masks:
        # data_part = frame_array[:, :].tolist()
        # frame_array[frame_array == 1] = 100
        # print("frame_array is {}".format(frame_array))
        timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")[:-3]
        file_name = "image.png"
        file_name_with_timestamp = f"{timestamp}_{file_name}"
        image_path = os.path.join("/home/ubuntu/Documents/EFS/Personal/ShuoShen/Track-Anything-Rosbag",
                                  file_name_with_timestamp)
        print("frame_id is {} image_path is {} ".format(frame_id, image_path))
        image = Image.fromarray(frame_array)
        image.save(image_path)

        # read local image.png
        processed_image = cv2.imread(image_path)
        # transfer image to png data
        success, encoded_image = cv2.imencode(".png", processed_image)
        base64_image_data = base64.b64encode(encoded_image).decode("utf-8")
        json_object = {
            "cv_mat": base64_image_data,
            "frame_id": frame_id
        }
        json_data.get("result").append(json_object)
        frame_id = frame_id + 1
        os.remove(image_path)

    return jsonify(json_data), 200


# nohup python  lde_online_service.py > /dev/null 2>&1 &
if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000, debug=True, threaded=False, processes=2)
    app.run(host='0.0.0.0', port=5000, debug=True)
