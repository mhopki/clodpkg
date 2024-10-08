#!/usr/bin/env python3

# use rgb pixels as arguments 
# convert rgb to hsc camera pixels 
# pixels to mirror values 
# run mirror and camera capture 

import rospy
import NanoPython.nano as nano
import optoKummenberg
import optoMDC
from mirrorCoordinates import *
import numpy as np
import argparse 
import time 
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray

global yolo_x, yolo_y, pub
yolo_x = 100
yolo_y = 100

def capture(data):
    if data.data[0] != 0: #Make sure this callback is exited if any number other than 0 is received, 0 means take picture
        print("NOT PHOTO")
        return None

    global yolo_x, yolo_y, pub
    x = int(2448/2)
    y = int(2048/2)
    exposure = 100
    if yolo_x: #set the x to the center coordinate x
        x = yolo_x #int(args_x)
        print("yolo_x: ", x)
    if yolo_y:  #set the y to the center coordinate y
        y =  yolo_y #int(args_y)
        print("yolo_y: ", y)
    exposure = 100#int(args_exposure) 

    # connect and initialize cam
    # Check connection with configure
    sensor = nano.Nano()
    sensor.connect()

    # connect mirror 
    mre2 = optoMDC.connect()

    # initiate x and y channels 
    ch_0 = mre2.Mirror.Channel_0
    ch_0.StaticInput.SetAsInput()                        
    ch_0.InputConditioning.SetGain(1.0)                  
    ch_0.SetControlMode(optoMDC.Units.XY)           
    ch_0.Manager.CheckSignalFlow() 

    ch_1 = mre2.Mirror.Channel_1
    ch_1.StaticInput.SetAsInput()                        
    ch_1.InputConditioning.SetGain(1.0)                 
    ch_1.SetControlMode(optoMDC.Units.XY)          
    ch_1.Manager.CheckSignalFlow() 

   
    resmatrix = np.array([[ 1.66472963e-01, -4.21643661e-03,  1.15935353e+03],
			[ 1.52134463e-02, -1.69968630e-01,  1.49403021e+03],
  			[ 3.98031468e-06, -6.06312963e-06,  1.00000000e+00]])


    rgb_cord = [x,y,1]
    print("rgb_coord: ", rgb_cord)

    hsc_cord = np.linalg.inv(resmatrix) @ rgb_cord
    print('hsc: ', hsc_cord)
    hsc_cord[0] = int(hsc_cord[0])
    hsc_cord[1] = int(hsc_cord[1])


    # script to take pixel value from beast and convert it to mirror coordinates
    # dimensions of the hyperspectral image is 640 x 640
    imagex = 640 
    imagey = 640 
    exposure = 100

    framePeriod = exposure
    maxFramesPerCube = imagey
    captureTime = (framePeriod * maxFramesPerCube * 1e-3) 
    mirror = np.zeros([1450,2])
    sampleSpeed = (1450*1000)/(exposure*imagey)
    sensor.configure(exposure=exposure, framePeriod=framePeriod)

    # select and if necessary adapt the code below to match your geometry

    # Mirror
    d = 0 # distance center of rotation to mirror surface, mm
    r_C = np.array([0, 0, d]) # center of rotation, origin

    # incoming beam in yz plane
    n_0 = normalize(np.array([0,-1,1]))
    r_OP_0 = np.array([0,1, -1])

    # target plane
    D = 9460 # Distance mirror center (undeflected) to target plane, mm

    alpha = np.deg2rad(45)
    A_TI = np.array([[ 1, 0 ,0], 
                    [ 0, np.cos(alpha), -np.sin(alpha)], 
                    [ 0, np.sin(alpha), np.cos(alpha)]])
    A_IT = np.transpose(A_TI)
    n_t = np.dot(A_IT, np.array([0,0,1]))
    r_OT = np.dot(A_IT, np.array([0,0,-D]))


    
    pixelx, pixely = (hsc_cord[0], hsc_cord[1])   

    top = pixely + 320      
    bottom = pixely - 320  

    y_target = np.expand_dims(np.linspace(top,bottom,1450),axis=1)
    x_target = np.ones_like(y_target)*pixelx

    xy = np.concatenate((x_target,y_target),axis=1)
    for i, (x_t, y_t) in enumerate(zip(x_target, y_target)):
        if i == 0 or i == len(x_target) - 1:
            print(x_t, y_t)
        n = getNormalVectorFromTargetXY_d0(x_t, y_t, A_IT, D, n_0)
        x,y = getXYFromNormalVector(n, 0, 90) # D is arbitrary for d==0
        mirror[i,:] = x,y

    vector=np.concatenate((mirror[:,0], mirror[:,1]), axis=0)


    static = ch_0.StaticInput
    static.SetAsInput()
    static.SetXY(0)
    static = ch_1.StaticInput
    static.SetAsInput()
    static.SetXY(0)

    mre2.VectorPatternMemory.SetPattern(index=0,vector=vector)

    vpu0 = ch_0.VectorPatternUnit
    vpu0.SetAsInput()
    vpu0.SetStart(0)
    vpu0.SetEnd(1450)
    vpu0.SetFreqSampleSpeed(sampleSpeed)
    vpu0.SetUnit(optoMDC.UnitType.XY)

    vpu1 = ch_1.VectorPatternUnit
    vpu1.SetAsInput()
    vpu1.SetStart(1451)
    vpu1.SetEnd(2900)
    vpu1.SetFreqSampleSpeed(sampleSpeed)
    vpu1.SetUnit(optoMDC.UnitType.XY)

    captureResponse = sensor.capture(prefix="sensor_image",maxFramesPerCube=maxFramesPerCube)

    mre2.set_value([vpu0.run, vpu1.run], [1, 1])

    time.sleep(captureTime)

    mre2.set_value([vpu0.run, vpu1.run], [0, 0])
    stoppedCapture = sensor.stopCapture()

    print("Capturing into folder:" + captureResponse["folder"])
    time.sleep(1)
    static = ch_0.StaticInput
    static.SetAsInput()
    static.SetXY(0)
    static = ch_1.StaticInput
    static.SetAsInput()
    static.SetXY(0)

    photo_msg = Float32MultiArray(data=[1])
    pub.publish(photo_msg)



def yolo_callback(data):
    xxx = 0
    #global yolo_x, yolo_y
    #yolo_x = data.detections[0].bbox.center.x - (data.detections[0].bbox.size_x)/2
    #yolo_y = data.detections[0].bbox.center.y - (data.detections[0].bbox.size_y)/2

def center_callback(data): #gives the center coordinates of the YOLO detection
    global yolo_x, yolo_y
    yolo_x = data.data[0]
    yolo_y = data.data[1]
    #print("center", yolo_x,yolo_y)


def spin():
    global pub

    rospy.init_node('hyperspectral_node', anonymous=True)
    rospy.Subscriber('/yolov7/yolov7', Detection2DArray, yolo_callback)
    rospy.Subscriber('/centerCoord', Float32MultiArray, center_callback)
    pub = rospy.Publisher('/take_photo', Float32MultiArray, queue_size=10)
    rospy.Subscriber('/take_photo', Float32MultiArray, capture)

    rospy.spin()


if __name__ == "__main__":
    spin()
