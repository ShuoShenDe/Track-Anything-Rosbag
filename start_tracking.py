import argparse
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

def save_data(filename, data):
    np.save(filename, data)
    print("Save data to {}".format(filename))
    
def get_reshape_size(org_x,org_y, ratio=1):
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
            # print(count)
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
                # print(len(image_list), len(image_list[0]), len(image_list[0][0]))
                # cv2.imwrite("out/{:03d}.jpg".format(count), new_cv_image)
                
            if count >= frame_end_id:
                break
            count+=1
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

def start_tracking(source_name, img_topic, mask,  template_mask, frame_start_id=0, frame_end_id=None, device = "cuda:0"):
    xmem_checkpoint = prepare_checkpoint()
    xmem = BaseTracker(xmem_checkpoint, device=device)
    xmem.clear_memory()
    # print(template_mask.shape)
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
            print(following_frames[i].shape, template_mask.shape)
            mask, logit, painted_image = xmem.track(following_frames[i], template_mask)
            masks.append(mask)
            logits.append(logit)
            painted_images.append(painted_image)   
        else:
            mask, logit, painted_image = xmem.track(following_frames[i])
            masks.append(mask)
            logits.append(logit)
            painted_images.append(painted_image)
    
    # print("painted_images: ",len(painted_images), len(painted_images[0]), len(painted_images[0][0]), len(painted_images[0][0][0]))
    return masks, logits, painted_images

def print_uniq_value(mat):
    row_num=0
    for row in mat:
        if any(item != 0 for item in row):
           print(row_num, set(row))
           row_num+=1


if __name__ == "__main__" :
    parser = argparse.ArgumentParser(description="Start tracking with specified bagpath and frameid.")
    parser.add_argument("--path", type=str, required=True, help="Path to the bag file.")  # "/home/qing/Documents/ShuoShen/Track-Anything/test_sample/Compass_2D_demo_allTopic_with_sam.bag"
    parser.add_argument("--frame_start_id", type=int, required=True, help="Frame ID for tracking.")
    parser.add_argument("--mask", type=str, required=False, help="Path to the mask file.")
    parser.add_argument("--topic", type=str, required=False, default="/tri_52", help="Image topic for tracking.")
    parser.add_argument("--frame_end_id", type=str, required=False, default="None", help="Frame End ID for tracking.")
    parser.add_argument("--mask_save_path", type=str, required=False, default="./masks_result.npy", help="mask_save_path")
    # Parse the arguments
    args = parser.parse_args()

    if not args.mask:
        template_mask = load_data("masks_result.npy")
        template_mask[template_mask == 100] = 79
    else:
        template_mask = load_data(args.mask)
        
    # Convert "None" string to None object
    if args.frame_end_id == "None":
        args.frame_end_id = None
    elif int(args.frame_end_id):
        args.frame_end_id = int(args.frame_end_id)
    else:
        args.frame_end_id = None
        print("frame_end_id {} is not a number, set to None".format(args.frame_end_id))

    print(args.path,  args.topic, template_mask, template_mask, args.frame_start_id, args.frame_end_id)
    masks, logits, painted_images = start_tracking(args.path,  args.topic, template_mask, template_mask, frame_start_id = args.frame_start_id, frame_end_id=args.frame_end_id)
    if args.mask_save_path.endswith(".npy"):       
        save_data(args.mask_save_path, template_mask)
    else:
        raise Exception("mask_save_path need end with .npy")
    
    row_num = 0
    # print(len(masks), masks[0].shape)
