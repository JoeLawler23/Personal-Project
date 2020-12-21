import cv2
import RPi.GPIO as GPIO
import numpy as np
from time import sleep

# GENERAL SETTINGS
#path = 'D:/Documents/Projects/Saffron Project/images/'
path = '/home/pi/Documents/Personal-Project/images/'
imageSize = 1920, 1080  # resolution that find_pistil_center analyzes pictures
outputSize = 500, 500  # resolution of debug window

# PISTIL SETTINGS
groupSizePistil = 80  # number of pixels in a group
pistilLower = np.array([0, 60, 60])  # lower bound for yellow selection (BGR)
pistilUpper = np.array([30, 255, 255])  # upper bound for yellow selection (BGR)

# STAMEN SETTINGS 
groupSizeStamen = 80  # number of pixels in a group
stamenLower = np.array([50, 25, 50])  # lower bound for yellow selection (BGR)
stamenUpper = np.array([255, 255, 255])  # upper bound for yellow selection (BGR)


# returns the position of the pistil in the frame
# pass the file path, and debugging boolean
# TODO FINISH
def harvestable(name, debug):
    # setup
    image = cv2.imread(path + name)  # get image from file
    image = cv2.resize(image, imageSize)  # resize image
    original = image.copy()  # make a copy
    mask = cv2.inRange(image, stamenLower, stamenUpper)  # make a mask image only containing colors within the range

    # draw rectangles over all groups from mask
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36, 255, 12), 2)

    # remove outliers from rectangles
    filteredContours = []
    for c in contours:
        if len(c) > groupSizeStamen:
            filteredContours.append(c)

    # output image with rectangles
    if debug:
        cv2.imshow('mask', cv2.resize(mask, outputSize))
        cv2.imshow('original', cv2.resize(original, outputSize))
        cv2.waitKey()

    # return the center
    return False


# returns the position of the pistil in the frame
# pass the file path, and debugging boolean
def pistil_center(name, debug):
    # setup
    image = cv2.imread(path + name)  # get image from file
    image = cv2.resize(image, imageSize)  # resize image
    original = image.copy()  # make a copy
    mask = cv2.inRange(image, pistilLower,
                       pistilUpper)  # make a mask image only containing colors within the range passed

    # draw rectangles over all groups from mask
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36, 255, 12), 2)

    # remove outliers from rectangles
    filteredContours = []
    for c in contours:
        if len(c) > groupSizePistil:
            filteredContours.append(c)

    # create large rectangle from list without outliers
    minX = 10000
    minY = 10000
    maxX = 0
    maxY = 0
    width = 0
    height = 0
    for c in filteredContours:
        x, y, w, h = cv2.boundingRect(c)
        if x < minX:
            minX = x
        if y < minY:
            minY = y
        if x + w > maxX:
            maxX = x + w
        if y + h > maxY:
            maxY = y + h
        width += w
        height += h

    # find the center
    center = int((maxX - minX) / 2 + minX), int((maxY - minY) / 2 + minY)
    centerOffset = int((maxX - minX) / 2 + minX + imageSize[0] / 100), int(
        (maxY - minY) / 2 + minY + imageSize[1] / 100)

    # output image with rectangles
    if debug:
        cv2.rectangle(original, (minX, minY), (maxX, maxY), (255, 255, 255), 2)
        cv2.rectangle(original, center, centerOffset, (255, 255, 255), -1)
        cv2.rectangle(mask, (minX, minY), (maxX, maxY), (255, 255, 255), 2)
        cv2.rectangle(mask, center, centerOffset, (255, 255, 255), -1)
        cv2.imshow('mask', cv2.resize(mask, outputSize))
        cv2.imshow('original', cv2.resize(original, outputSize))
        cv2.waitKey()

    # return the center
    return center


# takes picture from webcam given camera number
# saves to path with the name passed
def take_pic(name, camNum):
    cam = cv2.VideoCapture(camNum)
    img_name = path + name + ".jpg"
    ret, frame = cam.read()
    cv2.imwrite(img_name, frame)
    cam.release()
    cv2.destroyAllWindows()
    return

# raspberry pi init
def raspi_init():
    GPIO.setmode(GPIO.BOARD)
    return

GPIO.setup(3,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(7,GPIO.OUT)

if __name__ == '__main__':
    GPIO.output(3,GPIO.HIGH)
    GPIO.output(5,GPIO.LOW)
    sleep(2)
    pwm=GPIO.PWM(7,50)
    pwm.ChangeDutyCycle(2)
    pwm.start(1)
    sleep(2)
    pwm.stop()
    GPIO.cleanup()