import time

import serial
import serial.tools.list_ports

import numpy as np
import cv2
import math

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
        #print(res)
        a = res
    if len(res) < len(temp):
        return False
    for i in range(len(temp)):
        if ord(temp[i]) != res[i]:
            return False
    return True

pos1 = 0
minC =0
maxC=0
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
            ser = serial.Serial('/dev/cu.wchusbserial57350018291', 115200)
            #s3!!!!
            #ser = serial.Serial('/dev/cu.wchusbserial57340450811', 115200)
            #ser = serial.Serial('/dev/cu.wchusbserial57350018291', 115200) #s2
            while 1:
                # Читаем ответ от Arduino через Serial порт
                response = ser.readline()
                #пакет должен начинаться с temp = "0 255 0 255\r\n"
                while not checkRes(response):
                    response = ser.readline()

                len = int(ser.readline().strip())

                #pos = int(ser.readline().strip())
                aec = int(ser.readline().strip())
                #print(" aec:", aec)
                buff1 = ser.read(800)
                # 1 line из данных с esp
                img1x = np.zeros((1, 800), np.uint8)
                for j in range(800):
                    img1x[0, j] = buff1[j]

                img1x = cv2.resize(img1x, (1600, 60))

                img2 = np.asarray(img1x)
                if (len<100000):
                    buff = ser.read(len)
                    #print("len ", len)
                    image_np = np.frombuffer(buff, np.uint8)
                    img_np = cv2.imdecode(image_np, cv2.IMREAD_GRAYSCALE)
                    # была картинка 800х16 - увеличиваем в 1600х32
                    img_np = cv2.resize(img_np, (img_np.shape[1]*2,  img_np.shape[0]*2))
                    cv2.line(img_np, (20*2, 16), (780*2, 16), (255, 255, 255), 1)
                    #читаем сериал
                    s = ser.readline().strip()
                    par = s.split()
                    #print(s)
                    if par[0] == b'camera':
                        s = ser.readline().strip() #читаем позицию, которую esp определела
                        newP = int(par[1])
                        diff = int(par[2])
                        #print("diff:" + str(par[2]))
                        res = par[3].strip()

                        if res == b'ok':
                            pos1 = newP
                            print("======================================")
                            #print(math.degrees(math.atan(pos1/(751-pos1))))#S3
                            #print(math.degrees(math.atan((805- pos1) / (pos1-64))))#s2
                            '''
                            t = pos1 * 2
                            c = img2[0, t]
                            while int(img2[0, t]) >= c-diff//2 and t > 2:
                                if int(img2[0, t]) > c:
                                    c = int(img2[0, t])
                                t -= 2
                            minC = t
                            t = pos1 * 2
                            while img2[0, t] >= c-diff//2 and t < 770 * 2:
                                if int(img2[0, t]) > c:
                                    c = int(img2[0, t])
                                t += 2
                            maxC = t
                            '''
                        else:
                            #позиция с меткой error с esp
                            cv2.circle(img2, (newP*2, 30), 3, (100, 0, 0), 2)
                        #последняя хорошая позиция OK
                        cv2.circle(img2, (pos1*2, 30), 6, (0, 0, 0), 2)

                        cv2.line(img2, (minC, 10), (maxC, 10), (255, 255, 255), 2)
                        cv2.line(img2, (minC, 12), (maxC, 12), (0, 0, 0), 2)
                    print("pos:", pos1)


                    #imgBig = cv2.add(img_np, img2)
                    cv2.imshow("result", img_np)
                    cv2.imshow("1", img2)
                    cv2.waitKey(1)

                    ser.reset_input_buffer()

        except:
            print("No COM connection")
            time.sleep(1)

    ser.close()

#jpeg файл - тестовая функция
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
