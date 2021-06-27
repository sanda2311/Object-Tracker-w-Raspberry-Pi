from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import imutils
import time
import cv2
import RPi.GPIO as gpio
import serial

ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
ser.flush()

gpio.setmode(gpio.BOARD)
servo =35
gpio.setup(servo,gpio.OUT)
pwm = gpio.PWM(servo,50)
pwm.start(7)

camera = PiCamera()
camera.resolution = (400, 400)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(400, 400))

time.sleep(0.1)

def servo_ard(a):
    angle = "{}\n".format(a).encode('utf-8')
    ser.write(angle)
    time.sleep(0.02)

def servo(angle):
    dutyCycle = angle/18. + 2.
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep(0.02)


CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
#print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe('/home/pi/Desktop/Object detect/MobileNetSSD_deploy.prototxt.txt','/home/pi/Desktop/Object detect/MobileNetSSD_deploy.caffemodel')


angle = 100
servo_ard(angle)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array


    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image,0.007843, (400, 400), 127.5)

    net.setInput(blob)
    detections = net.forward()


    for i in np.arange(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]

        if confidence > 0.5:
            
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            #label = "{}: {:.2f}%".format(CLASSES[idx],confidence * 100)
            centreX = int((startX+endX)/2)
            centreY = int((startY+endY)/2)
            setPoint = int(w/2)
            
            label = CLASSES[idx]
            if label == 'person':
                cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                #cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
                #cv2.line(image,(centreX,0),(centreX,h),(255,255,255),2)
                #cv2.line(image,(0,centreY),(w,centreY),(255,255,255),2)
                if centreX < 150:
                    #angle = angle + 2
                    angle = 200 if angle > 200 else angle + 10
                    servo_ard(angle)
                elif centreX > 250:
                    angle = 0 if angle < 0 else angle - 10
                    servo_ard(angle)
                #print("setPoint: {} and centreX: {} and angle: {}".format(setPoint,centreX,angle))  

    # show the output frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
cv2.destroyAllWindows()
camera.close()