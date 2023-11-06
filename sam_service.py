from flask import Flask, request, jsonify, send_file
from start_tracking import start_tracking
import numpy as np
import json
import base64
import cv2

app = Flask(__name__)


def load_data(filename):
    return np.load(filename)


@app.route('/get_cvmat_by_sam', methods=['POST'])
def get_cvmat_by_sam():
    json_data = request.get_json()
    # if 'cv_mat' not in json_data:
    #     return jsonify({'error': 'Missing required parameters cv_mat, pls check'}), 400
    source = json_data.get('source',
                           "/home/ubuntu/Documents/EFS/Personal/ShuoShen/Track-Anything-Rosbag/test_sample/Compass_2D_demo_allTopic_with_sam.bag")
    topic = json_data.get('topic', "CAM_FRONT")
    cv_mat = json_data.get('cv_mat', load_data(
        "/home/ubuntu/Documents/EFS/Personal/ShuoShen/Track-Anything-Rosbag/test_sample/frame1_mask.npy"))

    # transfer json to numpy
    if json_data.get('cv_mat'):
        # decode Base64 data
        decoded_image_data = base64.b64decode(json_data.get('cv_mat'))
        # transfer decoded_image_data to numpy
        cv_mat = np.frombuffer(decoded_image_data, np.uint8)
        print("get cv_mat from client is {}".format(cv_mat))

    tracking_frame_start = json_data.get('tracking_frame_start', "0")
    tracking_frame_end = json_data.get('tracking_frame_end', "10")
    print("cv_mat is {}".format(cv_mat))
    masks, logits, painted_images = start_tracking(source, topic, cv_mat, cv_mat, int(tracking_frame_start),
                                                   int(tracking_frame_end))
    print("masks is {}".format(masks))

    json_data = {
        "result": []
    }
    frame_id = int(tracking_frame_start)
    for frame_array in masks:
        data_part = frame_array[:, :].tolist()
        json_object = {
            "cv_mat": data_part,
            "frame_id": frame_id
        }
        json_data.get("result").append(json_object)
        frame_id = frame_id + 1
        # if frame_id >= 2:
        #     break

    # print(json_data)
    return jsonify(json_data), 200


# nohup python  lde_online_service.py > /dev/null 2>&1 &
# pip install rospkg jira flask
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
