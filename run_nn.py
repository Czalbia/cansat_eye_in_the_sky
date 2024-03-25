from ultralytics import YOLO
import cv2
import os
import random
import pandas as pd
from PIL import Image
import torch
from ast import literal_eval
from PIL import ImageDraw
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time

data = {

}
results = pd.DataFrame()

photo_folder = "photo_folder"
model = YOLO("MODELS/long_range2.pt")

def convert_tensor_to_table(tensor):
    final_result = str(tensor)[7:-1]

    table = literal_eval(final_result)
    return table


def convert_tensor_to_table(tensor):
    final_result = str(tensor)[7:-1]

    table = literal_eval(final_result)
    return table


def detect(img_path):
    # Read the image
    img = cv2.imread(img_path)
    # print(img)

    # Pass the image through the detection model and get the result
    detect_result = model(img, conf=0.5)

    # Plot the detections
    detect_img = detect_result[0].plot()

    # Convert the image to RGB format
    detect_img = cv2.cvtColor(detect_img, cv2.COLOR_BGR2RGB)
    for r in detect_result:


        result_tensor_xyxy = r.boxes.xyxy
        result_tensor_conf = r.boxes.conf
    xyxy = convert_tensor_to_table(result_tensor_xyxy)
    conf = convert_tensor_to_table(result_tensor_conf)
    for i in range(len(xyxy)):
        xyxy[i].append(conf[i])
    return detect_img, xyxy


checked = []
print("\n\n\n\n")
counter = 0
while True:
    photo_files = [os.path.join(photo_folder, f) for f in os.listdir(photo_folder) if
                   os.path.isfile(os.path.join(photo_folder, f))]
    for sample_photo in photo_files:
        if sample_photo not in checked:
            print(sample_photo)
            # print(photo_files)
            # print(checked)
            detected_image, coords = detect(sample_photo)
            ready_image = Image.fromarray(detected_image)
            checked.append(sample_photo)

            df = pd.DataFrame(coords)
            if os.path.isdir(f"./RESULTS/results_{sample_photo[13:-4]}") == False:
                os.mkdir(f"./RESULTS/results_{sample_photo[13:-4]}")
            df.to_csv(f"./RESULTS/results_{sample_photo[13:-4]}/data.csv", index=False,header=["X1","Y1","X2","Y2","CONF"])
            ready_image.save(f"./RESULTS/results_{sample_photo[13:-4]}/image.jpg")

counter += 1
# ready_image.show()
print()
print("\n\n")
