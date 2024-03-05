import os
from ultralytics import YOLO
import cv2
from PIL import Image

# 使用训练好的yolo模型对图像进行推理
def predict(img):
    model = YOLO("best.pt")
    result = model(img)
    return result

# 将webot中的camera数据转化为yolo模型可以处理的数据
def img_formatting(data,width,height):
    image = Image.new("RGBA",(width, height))
    pixels = []
    for i in range(0, len(data),4):
        pixel = tuple(data[i:i+4])
        pixels.append(pixel)
    image.putdata(pixels)

    return image

# 主封装函数
def img_process(camera):
    data = camera.getImage()
    width = camera.getWidth()  
    height = camera.getHeight()    
    img = img_formatting(data,width,height)
    return predict(img)
    