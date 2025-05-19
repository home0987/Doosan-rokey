import time
import serial
import requests
import numpy
from io import BytesIO
from pprint import pprint
import globals
import os

import cv2

ser = serial.Serial("/dev/ttyACM0", 9600)


# API endpoint
api_url = ""

dataset_dir = 'dataset'
output_dir = 'dataset_crop'

# crop_size

def get_img():
    """Get Image From USB Camera

    Returns:
        numpy.array: Image numpy array
    """

    cam = cv2.VideoCapture(0)

    if not cam.isOpened():
        print("Camera Error")
        exit(-1)

    ret, img = cam.read()
    cam.release()

    return img


def crop_img(img, size_dict):
    x = size_dict["x"]
    y = size_dict["y"]
    w = size_dict["width"]
    h = size_dict["height"]
    img = img[y : y + h, x : x + w]
    return img


def inference_reqeust(img: numpy.array, api_rul: str):
    """_summary_

    Args:
        img (numpy.array): Image numpy array
        api_rul (str): API URL. Inference Endpoint
    """
    _, img_encoded = cv2.imencode(".jpg", img)

    # Prepare the image for sending
    img_bytes = BytesIO(img_encoded.tobytes())

    # Send the image to the API
    files = {"file": ("image.jpg", img_bytes, "image/jpeg")}

    print(files)

    try:
        response = requests.post(api_url, files=files)
        if response.status_code == 200:
            pprint(response.json())
            return response.json()
            print("Image sent successfully")
        else:
            print(f"Failed to send image. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error sending request: {e}")

save_folder = 'dataset_crop'

save_folder = 'dataset'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

camera = cv2.VideoCapture(0)
import time


while 1:
    data = ser.read()

    if data == b"0":
        time.sleep(2)
        f, img = camera.read()    
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # 현재 시간을 기반으로 파일명 생성
        img_filename = os.path.join(save_folder, f"image_{timestamp}.jpg")
        cv2.imwrite(img_filename, img)
        time.sleep(1)
        
        ser.write(b"1")
        
        
    #     print("ice!!!")
        # img = get_img()
        # # crop_info = None
        # crop_info = {"x": 200, "y": 100, "width": 300, "height": 100}

        # if crop_info is not None:
        #     img = crop_img(img, crop_info)

        # timestamp = time.strftime("%Y%m%d_%H%M%S")
        # #save_folder = 'dataset_crop'
        # img_filename = os.path.join(save_folder, f"image_{timestamp}.jpg")
        # print(img_filename)
        # cv2.imwrite(img_filename, img)

        # cv2.imshow("", img)
        # cv2.waitKey(1)
        # result = inference_reqeust(img, api_url)
        # ser.write(b"1")
    else:
        pass
    
    



