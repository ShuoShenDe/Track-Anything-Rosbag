import cv2
from cv_bridge import CvBridge
import rosbag
from sensor_msgs.msg import CompressedImage  # Import CompressedImage instead of Image
from sensor_msgs.msg import Image
import glob
import os


def get_reshape_size(org_x,org_y, ratio=1):
    reshape_x = int(org_x*ratio)
    reshape_y = int(org_y*ratio)
    return reshape_x, reshape_y

def images_to_video(image_folder, video_path, fps):
    # Get all files from the folder
    images = sorted(glob.glob(os.path.join(image_folder, '*.jpg')))

    # Check if there are any images to process
    if len(images) == 0:
        print("No images found in the specified directory!")
        return

    # Read the first image to get the width and height
    img = cv2.imread(images[0])
    height, width, layers = img.shape
    print(height, width)
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    reshape_x, reshape_y = get_reshape_size(width, height)
    video = cv2.VideoWriter(video_path, fourcc, fps, (reshape_x, reshape_y))

    # Add images to video
    for i, image_file in enumerate(images):
        print("Processing image {} of {}".format(i+1, len(images)))
        if i >200:
            break
        img = cv2.imread(image_file)
        new_cv_image = cv2.resize(img, (reshape_x, reshape_y))
        video.write(new_cv_image)

    # Release the VideoWriter
    video.release()
    print("Video created successfully!")

def image_process(bag_path="test_sample/Compass_2D_demo_allTopic_with_sam.bag"):
    print("start")
    img_topic = "CAM_FRONT"
    # Initialize CvBridge
    bridge = CvBridge()
    count = 0
    with rosbag.Bag(bag_path, mode='r') as bag:
       
        for topic, msg, t in bag.read_messages(topics= [img_topic]):
            # Convert compressed image message to CV image
            img_msg = Image()
            img_msg.data = msg.compressed_data
            img_msg.header = msg.header
            bridge = CvBridge()
            cv_image = bridge.compressed_imgmsg_to_cv2(
                img_msg, "passthrough")            
            (org_y, org_x, z) = cv_image.shape
            
            reshape_x, reshape_y = get_reshape_size(
                        org_x, org_y)            
            new_cv_image = cv2.resize(cv_image, (reshape_x, reshape_y))
            print(count)
            # Save image
            cv2.imwrite("out/{:03d}.jpg".format(count), new_cv_image)
            count += 1


if __name__ == '__main__':
    image_process()
    # images_to_video('out/', 'denso_middle.mp4', 10)
