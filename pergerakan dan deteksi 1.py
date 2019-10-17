#!/usr/bin/env python
from __future__ import print_function
    
import roslib
roslib.load_manifest('koding')
import sys
import os
import rospy
import rosbag
import time
import cv2
import serial
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:    
  
  def __init__(self):
    self.height = 0
    self.subNavdata = rospy.Subscriber('/ardrone/navdata/',Navdata,self.ReceiveNavdata) 
    self.pubLand = rospy.Publisher('/ardrone/land',Empty, queue_size=10)
    self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=10)
    self.pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    self.command = Twist()
    self.bridge = CvBridge()
    rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    rospy.on_shutdown(self.land)

  def ReceiveNavdata(self,navdata):
      self.status = navdata.state
      self.height = navdata.altd
      self.kecepatanX = navdata.vx
      self.kecepatanY = navdata.vy
      self.kecepatanZ = navdata.vz
      rate = rospy.Rate(10) 
     
  def takeoff(self):
      self.pubTakeoff.publish(Empty())
      rate = rospy.Rate(10)
      time.sleep(1)

  def land(self):
      self.pubLand.publish(Empty())
      rate = rospy.Rate(10)
      time.sleep(1)

  def Takeoff():
      pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
      rospy.init_node('takeoff', anonymous=True)
      rate = rospy.Rate(10) # 10hz 
      while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()
  
  def Land():
      pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
      rospy.init_node('land', anonymous=True)
      rate = rospy.Rate(10) # 10hz 
      while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()

  def stop(self):
      self.command.linear.x=0.0
      self.command.linear.y=0.0
      self.command.linear.z=0.0
      self.command.angular.x=0.0
      self.command.angular.y=0.0
      self.command.angular.z=0.0
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
  
  def geser_kiri(self):
      self.command.linear.y=0.5  #0.025
      #rospy.sleep(5.0) delay respon sistem
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def geser_kanan(self): 
      self.command.linear.y=-0.5
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def maju(self):
      self.command.linear.x=0.5
      self.command.linear.y=0.0
      self.command.linear.z=0.0
      self.command.angular.x=0.0
      self.command.angular.y=0.0
      self.command.angular.z=0.0
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      rospy.sleep(1000)
      #time.sleep(1)

  def mundur(self):
      self.command.linear.x=-0.5
      self.command.linear.y=0.0
      self.command.linear.z=0.0
      self.command.angular.z=0.0
      self.command.angular.y=0.0
      self.command.angular.z=0.0
      self.pubCommand.publish(self.command)
      font = cv2.FONT_HERSHEY_SIMPLEX
      rate = rospy.Rate(10)
      rospy.sleep(1000)

  def terbang_atas(self):
      self.command.linear.x=0.0
      self.command.linear.y=0.0
      self.command.linear.z=0.3
      self.command.angular.x=0.0
      self.command.angular.y=0.0
      self.command.angular.z=0.0
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def terbang_bawah(self): 
      self.command.linear.z=-0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)
  
  def rotasi_kiri(self):
      self.command.angular.z=0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def rotasi_kanan(self):  
      self.command.angular.z=-0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def hover(self):
      self.command.angular.x=0.0
      self.command.angular.y=0.0
      self.command.angular.z=0.0
      rate = rospy.Rate(10)
      time.sleep(1)

  def diagonal_kanan_bwh(self):
      self.command.linear.y=0.05
      self.command.linear.z=0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def diagonal_kiri_bwh(self):
      self.command.linear.y=-0.05
      self.command.linear.z=0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def diagonal_kanan_atas(self):
      self.command.linear.y=0.05
      self.command.linear.z=-0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def diagonal_kiri_atas(self):
      self.command.linear.y=-0.05
      self.command.linear.z=-0.05
      self.pubCommand.publish(self.command)
      rate = rospy.Rate(10)
      time.sleep(1)

  def zig_zag_kiri(self):  #mulai dari kiri
      print("gerakan zig-zag kiri")
      self.takeoff()
      rospy.sleep(0.02)
            
      self.maju()
      rospy.sleep(0.02)


  def zig_zag_kanan(self):
      print("gerakan zig-zag kanan")
      self.takeoff()
      rate = rospy.Rate(10)
      rospy.sleep(2000)
      self.stop()
      rospy.sleep(500)
      self.geser_kanan()
      rospy.sleep(2000)
      self.stop()
      rospy.sleep(500)
      self.maju()
      rospy.sleep(2000)
      self.stop()
      rospy.sleep(500)
      self.geser_kiri()
      rospy.sleep(2000)
      self.stop()
      rospy.sleep(500)
      self.maju()
      rospy.sleep(2000)
      self.stop()

  def SetCommand(self,linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
      self.command.linear.x = linear_x
      self.command.linear.y = linear_y
      self.command.linear.z = linear_z
      self.command.angular.x = angular_x
      self.command.angular.y = angular_y
      self.command.angular.z = angular_z
  
  def pid(self):
      print("mengontrol pergerakan drone berdsarkan posisi letak objek dari frame")

  def hitungjarak(self, knownWidth, focalLength, perWidth): 
      return (knownWidth * focalLength) / perWidth

  def nothing(self,x):
      pass

  def callback2(self,data):
  
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def callback(self,data):
    #time1 = 0
    #e1 = cv2.getTickCount()
    #e3 = str(long(e1))
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows, cols, channels) = cv_image.shape  
    imageWidth=cv_image.shape[1]
    imageHeight=cv_image.shape[0]
    cv_image3 =cv_image.copy() 
    
    cv_image4 =cv_image.copy()
    (rows, cols, channels) = cv_image4.shape
    imageWidth=cv_image4.shape[1]
    imageHeight=cv_image4.shape[0]


    cv2.line(cv_image4,(int(imageWidth/25*8),0),(int(imageWidth/25*8),int(imageHeight)),(255,0,0),1)
    #cv2.line(cv_image,(int(imageWidth/25*12.5),0),(int(imageWidth/25*12.5),int(imageHeight)),(255,0,0),2)
    cv2.line(cv_image4,(int(imageWidth/25*17),0),(int(imageWidth/25*17),int(imageHeight)),(255,0,0),1)
    cv2.line(cv_image4,(int(0),int(imageHeight/25*17)),(int(imageWidth),int(imageHeight/25*17)),(255,0,0),1)
    #cv2.line(cv_image,(int(0),int(imageHeight/25*12.5)),(int(imageWidth),int(imageHeight/25*12.5)),(255,0,0),2)
    cv2.line(cv_image4,(int(0),int(imageHeight/25*8)),(int(imageWidth),int(imageHeight/25*8)),(255,0,0),1)

    cv2.line(cv_image,(int(imageWidth/25*8),0),(int(imageWidth/25*8),int(imageHeight)),(255,0,0),1)
    #cv2.line(cv_image,(int(imageWidth/25*12.5),0),(int(imageWidth/25*12.5),int(imageHeight)),(255,0,0),2)
    cv2.line(cv_image,(int(imageWidth/25*17),0),(int(imageWidth/25*17),int(imageHeight)),(255,0,0),1)
    cv2.line(cv_image,(int(0),int(imageHeight/25*17)),(int(imageWidth),int(imageHeight/25*17)),(255,0,0),1)
    #cv2.line(cv_image,(int(0),int(imageHeight/25*12.5)),(int(imageWidth),int(imageHeight/25*12.5)),(255,0,0),2)
    cv2.line(cv_image,(int(0),int(imageHeight/25*8)),(int(imageWidth),int(imageHeight/25*8)),(255,0,0),1)
    #cv2.line(cv_image,(int(imageWidth/25*12.5),0),(int(imageWidth/25*12.5),int(imageHeight)),(0,0,255),2)
    #cv2.line(cv_image,(int(0),int(imageHeight/25*12.5)),(int(imageWidth),int(imageHeight/25*12.5)),(0,0,255),2)

        
    tight= cv2.Canny(cv_image, 225, 250) 
   
    yuv = cv2.GaussianBlur(cv_image,(5,5),0)
    yuv2 = cv2.GaussianBlur(yuv,(3,3),0)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower1=np.array([0, 2, 250], np.uint8)
    higher1=np.array([49, 105, 255], np.uint8)
    hsv_mask = cv2.inRange(hsv, lower1, higher1)
 
    hsv_mask2 = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    res1 = cv2.bitwise_and(cv_image, cv_image, mask=hsv_mask)

    kernel = np.ones((1,1), np.uint8)
    erosion = cv2.dilate(hsv_mask, kernel, iterations=1)
    dilation = cv2.erode(hsv_mask, kernel, iterations=1)
   
    ycrcb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)
    lower2=np.array([225, 115, 0], np.uint8)
    higher2=np.array([255,218, 176], np.uint8)
    ycrcb_mask = cv2.inRange(ycrcb, lower2, higher2)

    ycrcb_mask2 = cv2.morphologyEx(ycrcb_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    res2 = cv2.bitwise_and(cv_image, cv_image, mask=ycrcb_mask)
    
    kernel = np.ones((1,1), np.uint8)
    erosion2 = cv2.dilate(ycrcb_mask, kernel, iterations=1)
    dilation2 = cv2.erode(ycrcb_mask, kernel, iterations=1)

    merge = cv2.bitwise_or(hsv_mask, ycrcb_mask)
    merge2 = cv2.bitwise_and(hsv_mask2, ycrcb_mask2)
    
    #cv2.imshow("tes3",merge2)
    kernel = np.ones((5,5),np.uint8)
    merge_mask = cv2.morphologyEx(merge, cv2.MORPH_OPEN, kernel)
    merge_mask = cv2.morphologyEx(merge_mask, cv2.MORPH_CLOSE, kernel)
    
    cnts = cv2.findContours(merge_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius)= cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        d = float(cx)
        e = float(cy)
        cv2.line(cv_image,(int(x),0),(int(x),360),(255,0,0),1)
        cv2.line(cv_image,(0,int(y)),(640,int(y)),(255,0,0),1)  
        cv2.drawContours(cv_image, cnts, -1, (0,255,0), 1)    
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image,'Api Terdeteksi',(10,200), font, 1, (200,255,155), 1, cv2.LINE_AA) 
        hitung_pixel = cv2.countNonZero(merge2)
                
        if radius > 2:
            cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(cv_image, center, 3, (0, 0, 255), -1)
            cv2.putText(cv_image,"Nyala Api", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5,(0, 0, 255),2)
            cv2.putText(cv_image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

            if int(x)>=0 and int(x)<=int(imageWidth/25*8):
               if int(y)>=int(imageHeight/25*17) and int(y)<=int(imageHeight):
                  print("Drone bergerak diagonal kiri bawah")
                  font = cv2.FONT_HERSHEY_SIMPLEX
                  cv2.putText(cv_image,"Gerak diagonal kiri bawah",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                   
                  #cv2.putText(cv_image,"Waktu sistem"+str(time1),(10,90), font, 0.7, (200,255,155), 2, cv2.LINE_AA)        

                  if hitung_pixel>=501 and hitung_pixel<=2000:
                     print('Jumlah pixel api :',hitung_pixel,'px')
                     print("Drone hover dan stop atau langsung landing")
                     #self.land()

                  elif hitung_pixel>=0 and hitung_pixel<=500:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone mendekat")
                       #self.maju()

                  elif radius>=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone menjauh")
                       #self.mundur()

            elif int(x)>=0 and int(x)<=int(imageWidth/25*17):
                 if int(y)>=int(imageHeight/25*17) and int(y)<=int(imageHeight):
                    print ("Drone bergerak ke atas")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak ke atas",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                    time1 = 0
                    e2 = cv2.getTickCount()
                    e3 = cv2.getTickFrequency()
                    time1 = (e2-e1)/cv2.getTickFrequency()+time1
                     
                    if hitung_pixel>=501 and hitung_pixel<=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone hover dan stop atau langsung landing")
                       #self.land()

                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()

            elif int(x)>=0 and int(x)>=int(imageWidth/25*17):
                 if int(y)>=int(imageHeight/25*17) and int(y)<=int(imageHeight):
                    print ("Drone bergerak diagonal kanan-bawah")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak diagonal kanan bawah",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
 
                    if hitung_pixel>=501 and hitung_pixel<=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone hover dan stop atau langsung landing")
                       #self.land()

                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()

            if int(x)>=0 and int(x)<=int(imageWidth/25*8):
               if int(y)>=int(imageHeight/25*8) and int(y)<=int(imageHeight/25*17):
                  print ("Drone bergerak ke kanan")
                  font = cv2.FONT_HERSHEY_SIMPLEX
                  cv2.putText(cv_image,"Gerak yaw",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                  time1 = 0
                  e2 = cv2.getTickCount()
                  e3 = cv2.getTickFrequency()
                  time1 = (e2-e1)/cv2.getTickFrequency()+time1
                  #print("Waktu sistem : "+str(time1))
                  #cv2.putText(cv_image,"Waktu sistem : "+str(time1)+" s",(10,50), font, 0.7, (200,255,155), 2, cv2.LINE_AA)

                  if hitung_pixel>=501 and hitung_pixel<=2000:
                     print('Jumlah pixel api :',hitung_pixel,'px')
                     print("Drone hover dan stop atau langsung landing")
                     #self.land()

                  elif hitung_pixel>=0 and hitung_pixel<=500:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone mendekat")
                       #self.maju()

                  elif radius>=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone menjauh")
                       #self.mundur()
         
            elif int(x)>=0 and int(x)<=int(imageWidth/25*17):
                 if int(y)>=int(imageHeight/25*8) and int(y)<=int(imageHeight/25*17):
                    print ("Drone tetap pada koordinat posisi")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak Pitch",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                    #font = cv2.FONT_HERSHEY_SIMPLEX ocv2.putText(cv_image,"Status : Gerak Maju",(10,50), font, 0.7, 
                    #e2 = cv2.getTickCount()
                    time1 = 0
                    e2 = cv2.getTickCount()
                    e3 = cv2.getTickFrequency()
                    #time1 = (e2-e1)/cv2.getTickFrequency()+time1
                    #print("Waktu sistem : "+str(time1))
                    #cv2.putText(cv_image,"Waktu sistem : "+str(time1)+" s",(10,50), font, 0.7, (200,255,155), 2, cv2.LINE_AA)

                    if hitung_pixel>=501 and hitung_pixel<=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone hover dan stop atau langsung landing")
                       #self.land()

                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #rospy.sleep(0.7)
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()
                   
            elif int(x)>=0 and int(x)>=int(imageWidth/25*17):
                 if int(y)>=int(imageHeight/25*8) and int(y)<=int(imageHeight/25*17):
                    print ("Drone bergerak ke kiri")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak yaw",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                    time1 = 0
                    e2 = cv2.getTickCount()
                    e3 = cv2.getTickFrequency()
                    time1 = (e2-e1)/cv2.getTickFrequency()+time1
                    #print("Waktu sistem : "+str(time1))
                    #cv2.putText(cv_image,"Waktu sistem : "+str(time1)+" s",(10,50), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
          
                    if hitung_pixel>=501 and hitung_pixel<=2000:
                      print('Jumlah pixel api :',hitung_pixel,'px')
                      print("Drone hover dan stop atau langsung landing")
                      #self.land()

                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()

            if int(x)>=0 and int(x)<=int(imageWidth/25*8):
               if int(y)>=0 and int(y)<=int(imageHeight/25*8):
                  print ("Drone bergerak diagonal kiri atas")
                  font = cv2.FONT_HERSHEY_SIMPLEX
                  cv2.putText(cv_image,"Gerak diagonal kiri atas",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)

                  if hitung_pixel>=501 and hitung_pixel<=2000:
                     print('Jumlah pixel api :',hitung_pixel,'px')
                     print("Drone hover dan stop atau langsung landing")
                     #self.land()

                  elif hitung_pixel>=0 and hitung_pixel<=500:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone mendekat")
                       #self.maju()

                  elif radius>=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone menjauh")
                       #self.mundur()
       
            elif int(x)>=0 and int(x)<=int(imageWidth/25*17):
                 if int(y)>=0 and int(y)<=int(imageHeight/25*8):
                    print ("Drone bergerak ke atas")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak ke atas",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)
                    time1 = 0
                    e2 = cv2.getTickCount()
                    e3 = cv2.getTickFrequency()
                    time1 = (e2-e1)/cv2.getTickFrequency()+time1
                    #print("Waktu sistem : "+str(time1))
                    #cv2.putText(cv_image,"Waktu sistem : "+str(time1)+" s",(10,50), font, 0.7, (200,255,155), 2, cv2.LINE_AA)

                    if hitung_pixel>=501 and hitung_pixel<=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("Drone hover dan stop atau langsung landing")
                       #self.land()
 
                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()

            elif int(x)>=0 and int(x)>=int(imageWidth/25*17):
                 if int(y)>=0 and int(y)<=int(imageHeight/25*8):
                    print ("Drone bergerak diagonal kanan-atas")
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(cv_image,"Gerak diagonal kanan atas",(10,30), font, 0.7, (200,255,155), 2, cv2.LINE_AA)

                    if hitung_pixel>=501 and hitung_pixel<=2000:
                       print('Jumlah pixel api :',hitung_pixel,'px')
                       print("drone hover dan stop atau langsung landing")
                       #self.land()

                    elif hitung_pixel>=0 and hitung_pixel<=500:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone mendekat")
                         #self.maju()

                    elif radius>=2000:
                         print('Jumlah pixel api :',hitung_pixel,'px')
                         print("Drone menjauh")
                         #self.mundur()
                 
    cv2.imshow("Contour Detection", cv_image)
    cv2.imshow("Contour Detection2", cv_image4)
    #cv2.imshow("Blurring Citra", yuv)
    cv2.imshow("Konversi ke HSV", hsv)
    cv2.imshow("HSV Mask", hsv_mask)
    cv2.imshow("Color Filtering HSV", res1)
    #cv2.imshow("Opening HSV",hsv_mask2)

    cv2.imshow("Konversi ke YCbCr", ycrcb)
    cv2.imshow("YCbCr Mask", ycrcb_mask)
    cv2.imshow("Color Filtering YCbCr", res2)
    #cv2.imshow("Opening YCbCr",ycrcb_mask2)
    cv2.imshow("Combine HSV dan YCbCr", merge2)

    #e2 = cv2.getTickCount()
    #time1 = (e2-e1)/cv2.getTickFrequency()+time1
    #print("waktu awal:"+str(e3))
    #print("waktu akhir:"+str(e4))
    #print("waktu respons sistem :"+str(time1))

    cv2.waitKey(3)
    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

    time1 = 0
    e2 = cv2.getTickCount()      
    e3 = cv2.getTickFrequency()
    time1 = (e2-e1)/cv2.getTickFrequency()+time1
    #print("Waktu respons sistem : "+str(time1))
    #print("Clock Cycle / detik : "+str(e3))   

def pilihgerak():
    os.system("clear")
    print("     NAVIGASI DAN KONTROL SISTEM DETEKSI API QUADCOPTER AR DRONE ")
    print("              DENGAN METODE COLOR FILTERING HSV DAN YCBCR        ")
    print("==============================")
    print("Pilih jenis pergerakan ?")
    print("1.Gerak Manual")
    print("2.Gerak Otomatis")
    print("==============================")

def menu_manual():
    os.system("clear")
    print("     NAVIGASI DAN KONTROL SISTEM DETEKSI API QUADCOPTER AR DRONE ")
    print("              DENGAN METODE COLOR FILTERING HSV DAN YCBCR        ")
    print("==============================")
    print("  T  L  X   |   9    8 ")
    print("            |   7    6  ")
    print("============|=================")
    print("  Q  W  E \n")
    print("  A  S  D   ")
    print("============")
    print("  U  J   O   ")
    print("  Y  K   P \n")
    print("Keterangan :")
    print("T = takeoff      | W = maju             | J = rotasi kiri")
    print("L = land         | S = mundur           | K = rotasi kanan")
    print("A = geser kiri   | Q = terbang atas     | U = ke kiri depan")
    print("D = geser kanan  | E = terbang bawah    | O = ke kanan depan")
    print("X = stop         | Y = ke kiri belakang | P = ke kanan belakang")
    print("6 = diagonal kanan bawah | 7 = diagonal kiri bawah ")
    print("8 = diagonal kanan atas  | 9 = diagonal kiri atas  ")       
    print("0 = menu awal")
  
def menu_otomatis():
    os.system("clear")
    print(" NAVIGASI DAN KONTROL SISTEM DETEKSI API QUADCOPTER AR DRONE ")
    print("          DENGAN METODE COLOR FILTERING HSV DAN YCBCR        ")
    print("==============================")
    print("  H  C  V   | V  X     ")
    print("            |          ")
    print("============|=================")
    print("Keterangan :")
    print("H = zig-zag kanan  | V = maju kecepatan konstan  ")
    print("C = zig-zag kiri   | X = Stop     ")
    print("0 = menu awal")

def takeoff1():
      pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
      rospy.init_node('takeoff', anonymous=True)
      rate = rospy.Rate(10) # 10hz 
      while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()

def land1():
      pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
      rospy.init_node('land', anonymous=True)
      rate = rospy.Rate(10) # 10hz 
      while not rospy.is_shutdown():
          pub.publish(Empty())
          rate.sleep()

def maju1():
   command = Twist()
   command.linear.x=0.03
   command.linear.y=0.0
   command.linear.z=0.0
   command.angular.x=0.0
   command.angular.y=0.0
   command.angular.z=0.0
   pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=10) 
   pubCommand.publish(command)
   print("Status : Gerak Maju")
   rate = rospy.Rate(10)
   time.sleep(1)


def main(args):
  #time1=0
  global e1
  e1 = cv2.getTickCount()
  ic = image_converter()
  

  rospy.init_node('image_converter', anonymous=True)
  try: 
       while not rospy.is_shutdown():
           pilihgerak()
           pilih = input ("Pilih Gerakan : ")
           print("\n")
           if pilih == 1:
              menu_manual() 
           elif pilih == 2:
                menu_otomatis()
           elif pilih ==3:
                print("keluar dari program")
           try: 
                while not rospy.is_shutdown(): 
                    print("Tekan tombol di keyboard boskuh :")
                    key=sys.stdin.read(1)
                    if (key == str('t')):
                       ic.takeoff()
                    elif (key == str('l')):
                          land1()
                    elif (key == str('0')):
                          pilihgerak()     	
                    elif (key == str('a')):
                          ic.geser_kiri()
                    elif (key == str('d')):
                          ic.geser_kanan()
                    elif (key == str('w')):
                          maju1()
                    elif (key == str('s')):
                          ic. mundur()
                    elif (key == str('q')):
                          ic.terbang_atas()
                    elif (key == str('e')):
                          ic.terbang_bawah() 
                    elif (key == str('j')):
                          ic.rotasi_kiri()
                    elif (key == str('k')):
                          ic.rotasi_kanan()
                    elif (key == str('h')):
                          ic.hover()
                    elif (key == str('x')):
                          ic.stop()
                    elif (key == str('c')):
                          ic.zig_zag_kanan()
                    elif (key == str('b')):
                          ic.zig_zag_kiri()
                    elif (key == str('v')):
                          square()
                    elif (key == str('u')):
                          ke_kiri()
                    elif (key == str('o')):
                          ke_kanan()
                    elif (key == str('6')):
                          ic.diagonal_kanan_bwh()
                    elif (key == str('7')):
                          ic.diagonal_kiri_bwh()
                    elif (key == str('8')):
                          ic.diagonal_kanan_atas()
                    elif (key == str('9')):
                          ic.diagonal_kiri_atas()
           except rospy.ROSInterruptException:
               pass
  except rospy.ROSInterruptException:
      pass          

  try:
    rospy.spin()
    #takeoff()
  except KeyboardInterrupt:	
    print("Shutting down")
  cv2.destroyAllWindows()
   
if __name__ == '__main__':
   main(sys.argv) 
   #main(sys.argv)
# Sistem Deteksi Api pada Quadcopter AR Drone menggunakan YCbCr Colour Space Model + SVM (support vector machine)

