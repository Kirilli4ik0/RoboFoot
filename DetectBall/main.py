import time

import serial
import serial.tools.list_ports

import numpy as np
import cv2

import argparse

def getPorts():
    # Получаем список доступных Serial портов
    ports = list(serial.tools.list_ports.comports())
    # Выводим информацию о каждом порте
    for port in ports:
        print(f"Name: {port.name}")
        print(f"Порт: {port.device}")
        print(f"Описание: {port.description}")
        print(f"Производитель: {port.manufacturer}\n")

temp = "0 255 0 255\r\n"
def checkRes(res):
    if len(res)< 30:
        print(res)
    if len(res) < len(temp):
        return False
    for i in range(len(temp)):
        if ord(temp[i]) != res[i]:
            return False
    return True

numBlock = 15
midBlackArr = [60]*(numBlock+1)
koff = 1.5
width = 780
block = width//numBlock
line = 8 #8+16 #8 # 67
show = 4 #20
def findBall(img):
    global midBlackArr
    global koff

    print(img.shape[1], img.shape[0])
    if img.shape[0] < line:
        return

    i = 0
    sumBL = 0
    numBL = 0
    sect = 0
    found = 0
    err = 0
    ballSection = -1

    while i < width:
        cur = img.data[line, i]
        if cur < midBlackArr[sect]:
            i += 1
            if i % block == 0:
                if numBL != 0 and ballSection != sect:
                    midBlackArr[sect] = int(sumBL * koff / numBL)
                    if midBlackArr[sect]<20:
                        midBlackArr[sect] = 20
                    else:
                        if midBlackArr[sect]>90:
                            midBlackArr[sect] = 90
                sumBL = 0
                numBL = 0
                sect = i // block
            else:
                numBL += 1
                sumBL += cur
        else:
            start = i
            i += 1
            cur = img.data[line, i]
            while cur >= midBlackArr[sect] and i < width:
                i += 1
                cur = img.data[line, i]
                if sect != i // block:
                    sumBL = 0
                    numBL = 0
                    sect = i // block
            if i-start > 10:
                ballSection = sect
                cv2.line(img, (start, line+show), (i, line+show), (0, 0, 0), 1)
                cv2.line(img, (start, line + show+ 3), (i, line + show+ 3), (255, 255, 255), 1)
                found += 1
                print("pos :" + str(start) + " " + str(i))
                if found > 1 and err == 0:
                    koff += 0.5
                    for k in range(numBlock):
                        if midBlackArr[k] < 110:
                            midBlackArr[k] += 5
                    i = 0
                    sumBL = 0
                    numBL = 0
                    sect = 0
                    found = 0
                    err = 1
    if found==0:
        koff = 1.5
    print(midBlackArr)
    cv2.line(img, (0, line), (width, line), (255, 255, 255), 1)




midBlack = 70
def findBall1(img):
    global midBlack
    print(img.shape[1], img.shape[0])
    if img.shape[0] < 90:
        return

    #cv2.line(img, (0, 20), (800, 20), (255, 255, 255), 2)

    #cv2.circle(img, (80, 80), 55, (0, 255, 0), -1)
    line = 66
    #prev = img.data[line, 20]
    i = 30
    sumBL = 0
    numBL = 0
    while i < 720:
        cur = img.data[line, i]
        if cur < midBlack:
            i += 1
            numBL += 1
            sumBL += cur
        else:
            start = i
            i += 1
            cur = img.data[line, i]
            while cur >= midBlack and i < 720:
                i += 1
                cur = img.data[line, i]
            if i-start > 10:
                cv2.line(img, (start, line+20), (i, line+20), (255, 255, 255), 2)

    #midBlack = int(sumBL/numBL)
    print(midBlack)
    cv2.line(img, (30, line), (720, line), (255, 255, 255), 1)
    cv2.imshow("result", img)
    #cv2.resizeWindow('image', 900, 900)
    cv2.waitKey(100)

pos1 = 0
def getImage():
    global pos1
    cv2.namedWindow("result", cv2.WINDOW_NORMAL)
    #img = np.zeros((800, 600), np.uint8)
    #cv2.imshow("Image", img)
    #cv2.waitKey(1)
    while 1:
        try:
            #KKKKKKKKKKK
            #ser = serial.Serial('/dev/cu.usbmodem57340449801', 115200)
            #s2!!!!
            #ser = serial.Serial('/dev/cu.wchusbserial57340449801', 115200)
            #s3!!!!
            #ser = serial.Serial('/dev/cu.wchusbserial57340450811', 115200)
            ser = serial.Serial('/dev/cu.wchusbserial57350018291', 115200)
            while 1:
                # Читаем ответ от Arduino через Serial порт
                response = ser.readline()
                #пакет должен начинаться с temp = "0 255 0 255\r\n"
                while not checkRes(response):
                    response = ser.readline()

                len = int(ser.readline().strip())

                #pos = int(ser.readline().strip())
                aec = int(ser.readline().strip())
                print(" aec:", aec)
                buff1 = ser.read(800)
                # 1 line
                img1x = np.zeros((1, 800), np.uint8)
                for j in range(800):
                    img1x[0, j] = buff1[j]

                img1x = cv2.resize(img1x, (800, 60))

                img2 = np.asarray(img1x)
                if (len<100000):
                    buff = ser.read(len)
                    print("len ", len)
                    image_np = np.frombuffer(buff, np.uint8)
                    img_np = cv2.imdecode(image_np, cv2.IMREAD_GRAYSCALE)
                    findBall(img_np) #img1x  img_np
                    img_np = cv2.resize(img_np, (1600, 64))
                    #читаем сериал
                    s = ser.readline().strip()
                    if s == b'camera':
                        s = ser.readline().strip() #читаем позицию, которую esp определела
                        newP = int(s.strip())
                        res = ser.readline().strip()
                        if res == b'ok':
                            pos1 = newP
                        else:
                            #позиция с меткой error с esp
                            cv2.circle(img2, (newP, 30), 3, (100, 0, 0), 2)
                        #последняя хорошая позиция OK
                        cv2.circle(img2, (pos1, 30), 6, (255, 0, 0), 2)
                    print("pos:", pos1)


                    #imgBig = cv2.add(img_np, img2)
                    cv2.imshow("result", img_np)
                    cv2.imshow("1", img2)
                    cv2.waitKey(10)

                    ser.reset_input_buffer()

        except:
            print("No COM connection")
            time.sleep(1)

    ser.close()

def readF():
    #image = cv2.imread('aaa.jpg')
    with open('aaa.jpg', 'rb') as file:
        binary_data = file.read()
        image_np = np.frombuffer(binary_data, np.uint8)
        img_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
        cv2.imshow("Image", img_np)
        cv2.waitKey(10)


    #cv2.imshow("Image", image)
    #cv2.waitKey(0)


getPorts()
getImage()
#readF()
#cv2.waitKey(0)
