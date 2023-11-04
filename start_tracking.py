from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from tqdm import tqdm
import os
import requests
from rosbags.highlevel import AnyReader

import sys
sys.path.append(sys.path[0]+"/tracker")
sys.path.append(sys.path[0]+"/tracker/model")
from tracker.base_tracker import BaseTracker

try: 
    from mmcv.cnn import ConvModule
except:
    os.system("mim install mmcv")


def load_data(filename):
    return np.load(filename)

def get_reshape_size(org_x,org_y, ratio=0.5):
    reshape_x = int(org_x *ratio)
    reshape_y = int(org_y*ratio)
    return reshape_x, reshape_y

def image_process(bag_path,  img_topic,  frame_start_id=0, frame_end_id=None):
    print("start")
    if frame_end_id == None:
        frame_end_id = float('inf')
    images = []
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == img_topic]
        count = 0
        for connection, timestamp, rawdata in reader.messages(connections=connections): 
            # print(connection.msgcount)
            print(count)
            if count >= frame_start_id:
                msg = reader.deserialize(rawdata, connection.msgtype)
                # print(msg.header)
                img_msg = Image()
                img_msg.header = msg.header
                img_msg.data = msg.compressed_data
                bridge = CvBridge()
                cv_image = bridge.compressed_imgmsg_to_cv2(img_msg, "passthrough")
                (org_y, org_x, z) = cv_image.shape   # 720 1920*3
                reshape_x, reshape_y = get_reshape_size(org_x, org_y)
                new_cv_image = cv2.resize(cv_image, (reshape_x, reshape_y))
                # Save image
                image_list = new_cv_image
                images.append(image_list)
                print(len(image_list), len(image_list[0]), len(image_list[0][0]))
                # cv2.imwrite("out/{:03d}.jpg".format(count), new_cv_image)
                count+=1
            if count >= frame_end_id:
                break
    return images

def download_checkpoint(url, folder, filename):
    os.makedirs(folder, exist_ok=True)
    filepath = os.path.join(folder, filename)

    if not os.path.exists(filepath):
        print("download checkpoints ......")
        response = requests.get(url, stream=True)
        with open(filepath, "wb") as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)
        print("download successfully!")

    return filepath

def prepare_checkpoint(xmem_checkpoint_url = "https://github.com/hkchengrex/XMem/releases/download/v1.0/XMem-s012.pth", folder = "./checkpoints", xmem_checkpoint = "XMem-s012.pth"):
    xmem_checkpoint = download_checkpoint(xmem_checkpoint_url, folder, xmem_checkpoint)
    return xmem_checkpoint

def start_tracking(source_name, img_topic, mask,  frame_start_id=0, frame_end_id=None, device = "cuda:0"):
    xmem_checkpoint = prepare_checkpoint()
    xmem = BaseTracker(xmem_checkpoint, device=device)
    xmem.clear_memory()

    template_mask = load_data("frame1_mask.npy")
    print(template_mask.shape)

    masks =[]
    logits = []
    painted_images = []

    if len(np.unique(template_mask))==1:
        template_mask[0][0]=1
        operation_log = [("Error! Please add at least one mask to track by clicking the left image.","Error"), ("","")]
        raise Exception(operation_log)
    following_frames = image_process(source_name, img_topic = img_topic,  frame_start_id = frame_start_id, frame_end_id=frame_end_id)
    for i in tqdm(range(len(following_frames)), desc="Tracking image"):
        if i ==0:           
            mask, logit, painted_image = xmem.track(following_frames[i], template_mask)
            masks.append(mask)
            logits.append(logit)
            painted_images.append(painted_image)   
        else:
            mask, logit, painted_image = xmem.track(following_frames[i])
            masks.append(mask)
            logits.append(logit)
            painted_images.append(painted_image)
    
    print("painted_images: ",len(painted_images), len(painted_images[0]), len(painted_images[0][0]), len(painted_images[0][0][0]))
    return masks, logits, painted_images


if __name__ == "__main__" :
    masks, logits, painted_images = start_tracking("/home/qing/Documents/ShuoShen/Track-Anything/test_sample/Compass_2D_demo_allTopic_with_sam.bag",  "CAM_FRONT", None, frame_end_id=10)