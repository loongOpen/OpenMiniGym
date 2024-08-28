import cv2
import mediapipe as mp
import math
import time
import numpy as np
from threading import Thread
import math
import socket
import struct
class handDetector1:
    """
    Finds Hands using the mediapipe library. Exports the landmarks
    in pixel format. Adds extra functionalities like finding how
    many fingers are up or the distance between two fingers. Also
    provides bounding box info of the hand found.
    """
 
    def __init__(self,mode = False,maxHands = 2,comp = 1,detectionCon = 0.5,trackCon = 0.5):#这里由于函数库更新，所以多了一个复杂度参数，默认设为1
        """
        :param mode: In static mode, detection is done on each image: slower
        :param maxHands: Maximum number of hands to detect
        :param detectionCon: Minimum Detection Confidence Threshold
        :param minTrackCon: Minimum Tracking Confidence Threshold
        """
        self.mode = mode
        self.maxHands = maxHands
        self.comp = comp
        self.detectionCon = detectionCon
        self.trackCon = trackCon
 
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode,self.maxHands,self.comp,
                                        self.detectionCon,self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
 
        self.handLmStyle = self.mpDraw.DrawingSpec(color=(0, 0, 255), thickness=5)  # 点的样式，前一个参数是颜色，后一个是粗细
        self.handConStyle = self.mpDraw.DrawingSpec(color=(0, 255, 0), thickness=3)  # 线的样式BGR，前一个参数是颜色，后一个是粗细

        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.tipIds_low = [2, 7, 11, 15, 19]
        self.fingers = []
        self.lmList = []
 
    def findHands(self, img, draw=True, flipType=True):
        """
        Finds hands in a BGR image.
        :param img: Image to find the hands in.
        :param draw: Flag to draw the output on the image.
        :return: Image with or without drawings
        """
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        allHands = []
        h, w, c = img.shape
        if  self.results.multi_hand_landmarks:
            for handType,handLms in zip(self.results.multi_handedness,self.results.multi_hand_landmarks):
                myHand={}
                ## lmList -- all hand
                mylmList = []
                xList = []
                yList = []
                for id, lm in enumerate(handLms.landmark):
                    px, py = int(lm.x * w), int(lm.y * h)
                    mylmList.append([px, py])
                    xList.append(px)
                    yList.append(py)
 
                ## bbox -- all hand 
                xmin, xmax = min(xList), max(xList)
                ymin, ymax = min(yList), max(yList)
                boxW, boxH = xmax - xmin, ymax - ymin
                bbox = xmin, ymin, boxW, boxH
                cx, cy = bbox[0] + (bbox[2] // 2), \
                         bbox[1] + (bbox[3] // 2)

                ## lmList -- all low finger
                mylmList_low = []
                xList_low = []
                yList_low = []
                low_list = [10,11,12, 14,15,16 ,  18,19,20]
                for id, lm in enumerate(handLms.landmark):
                    if id in list(low_list):
                        px, py = int(lm.x * w), int(lm.y * h)
                        mylmList_low.append([px, py])
                        xList_low.append(px)
                        yList_low.append(py)
 
                ## bbox -- all hand 
                xmin_low, xmax_low = min(xList_low), max(xList_low)
                ymin_low, ymax_low = min(yList_low), max(yList_low)
                boxW_low, boxH_low = xmax_low - xmin_low, ymax_low - ymin_low
                bbox_low = xmin_low, ymin_low, boxW_low, boxH_low
                cx_low, cy_low = bbox_low[0] + (bbox_low[2] // 2), \
                         bbox_low[1] + (bbox_low[3] // 2)
                 
                #---------------------------------------------------
                myHand["lmList"] = mylmList
                
                myHand["bbox"] = bbox
                myHand["center"] =  (cx, cy)

                myHand["bbox_lower"] = bbox_low
                myHand["center_lower"] =  (cx, cy)
                if flipType:
                    if handType.classification[0].label =="Right":
                        myHand["type"] = "Left"
                    else:
                        myHand["type"] = "Right"
                else:myHand["type"] = handType.classification[0].label
                allHands.append(myHand)
 
                ## draw
                if draw and myHand["type"] == "Right":#doghome
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
                    if 0:
                        cv2.rectangle(img, (bbox_low[0] - 20, bbox_low[1] - 20),
                                    (bbox_low[0] + bbox_low[2] + 20, bbox_low[1] + bbox_low[3] + 20),
                                    (0, 255, 0), 3)
                        cv2.circle(img, (cx_low, cy_low), 15, (0, 255, 0), cv2.FILLED)
                    if 0:
                        cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                                    (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                                    (255, 0, 255), 2)
                    cv2.putText(img,myHand["type"],(bbox[0] - 30, bbox[1] - 30),cv2.FONT_HERSHEY_PLAIN,
                                2,(255, 0, 255),2)
        if draw:
            return allHands,img
        else:
            return allHands
 
    def fingersUp(self,myHand):
        """
        Finds how many fingers are open and returns in a list.
        Considers left and right hands separately
        :return: List of which fingers are up
        """
        myHandType =myHand["type"]
        myLmList = myHand["lmList"]
        check_pix=50 #amao 14
        if self.results.multi_hand_landmarks:
            fingers = []
            # Thumb
            #print(myLmList[self.tipIds[0]][0],myLmList[self.tipIds[0] - 1][0])
            # if myHandType == "left":
            #     if abs(myLmList[self.tipIds[0]][0] - myLmList[self.tipIds[0] - 1][0])<check_pix:
            #         fingers.append(1)
            #     else:
            #         fingers.append(0)
            # else:
            #     if abs(myLmList[self.tipIds[0]][0] < myLmList[self.tipIds[0] - 1][0])<check_pix:
            #         fingers.append(1)
            #     else:
            #         fingers.append(0)
 
            # 4 Fingers
            for id in range(0, 5):
                dis,info=self.findDistance(myLmList[self.tipIds[id]],myLmList[self.tipIds_low[id]])

                if dis<check_pix:#abs(myLmList[self.tipIds[id]][1] < myLmList[self.tipIds[id] - 2][1])<check_pix:
                    fingers.append(1)
                else:
                    fingers.append(0)
        return fingers
    
    def handType(self):
        """
        检查传入的手部是左还是右
        ：return: "Right" 或 "Left"
        """
        if self.results.multi_hand_landmarks:
            if self.lmList[17][0] < self.lmList[5][0]:
                return "Right"
            else:
                return "Left"
        
     

    def findDistance(self,p1, p2, img=None):
        """
        Find the distance between two landmarks based on their
        index numbers.
        :param p1: Point1
        :param p2: Point2
        :param img: Image to draw on.
        :param draw: Flag to draw the output on the image.
        :return: Distance between the points
                 Image with output drawn
                 Line information
        """
 
        x1, y1 = p1
        x2, y2 = p2
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        length = math.hypot(x2 - x1, y2 - y1)
        info = (x1, y1, x2, y2, cx, cy)
        if img is not None:
            cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 15, (255, 0, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            return length,info, img
        else:
            return length, info
        

# 构造一个贪吃蛇移动的类
class SnakeGameClass:
 
    #（一）初始化
    def __init__(self):
 
        self.points = []  # 蛇的身体的节点坐标
        self.lengths = []  # 蛇身各个节点之间的坐标
        self.currentLength = 0  # 当前蛇身长度
        self.allowedLength = 150  # 没吃东西时，蛇的总长度
        self.previousHead = (0,0)  # 前一个蛇头节点的坐标
 
    #（二）更新增加蛇身长度
    def update(self, imgMain, currentHead): # 输入图像，当前蛇头的坐标
 
        px, py = self.previousHead  # 获得前一个蛇头的x和y坐标
        cx, cy = currentHead  # 当前蛇头节点的x和y坐标
        
        # 添加当前蛇头的坐标到蛇身节点坐标列表中
        self.points.append([cx,cy])
 
        # 计算两个节点之间的距离
        distance = math.hypot(cx-px, cy-py)  # 计算平方和开根
        # 将节点之间的距离添加到蛇身节点距离列表中
        self.lengths.append(distance)
        # 增加当前蛇身长度
        self.currentLength += distance
 
        # 更新蛇头坐标
        self.previousHead = (cx,cy)
 
        #（三）减少蛇尾长度，即移动过程中蛇头到蛇尾的长度不大于150
        if self.currentLength > self.allowedLength:
 
            # 遍历所有的节点线段长度。新更新的蛇头索引在列表后面，蛇尾的索引在列表前面
            for i, length in enumerate(self.lengths):
 
                # 从蛇尾到蛇头依次减线段长度，得到的长度是否满足要求
                self.currentLength -= length
 
                # 从列表中删除蛇尾端的线段长度，以及蛇尾节点
                self.lengths.pop(i)
                self.points.pop(i)
 
                # 如果当前蛇身长度小于规定长度，满足要求，退出循环
                if self.currentLength < self.allowedLength:
                    break
 
        #（四）绘制蛇
        # 当节点列表中有值了，才能绘制
        if self.points:
 
            # 遍历蛇身节点坐标
            for i, point in enumerate(self.points):  
                # 绘制前后两个节点之间的连线
                if i != 0:
                    cv2.line(imgMain, tuple(self.points[i-1]), tuple(self.points[i]), (0,255,0), 5)
 
            # 在蛇头的位置画个圆
            cv2.circle(imgMain, tuple(self.points[-1]), 5, (255,0,0), cv2.FILLED)
 
        # 返回更新后的图像
        return imgMain
 
volRange = [0,300]#]#volume.GetVolumeRange()
minVol = volRange[0]
maxVol = volRange[1]
vol = 0
volBar = 400
volPer = 0

#（3）找到手掌间的距离和实际的手与摄像机之间的距离的映射关系
# x 代表手掌间的距离(像素距离)，y 代表手和摄像机之间的距离(cm)
x = [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57]
y = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
coff = np.polyfit(x, y, 2)  #构造二阶多项式方程

volRange = [0,300]#]#volume.GetVolumeRange()
minVol = volRange[0]
maxVol = volRange[1]
vol = 0
volBar = 400
volPer = 0

size_hand = 0
size_hand_base= 22 #pix
size_to_dis =0.04
dis_hand_base=0.5 #meter

wCam,hCam = 640/1,480/1

flt_rate=0.8
posx_flt=0
posy_flt=0
posz_flt=0
cap_flt=0


tx_data_udp =  [0]*500
arm_pos_exp=[0,0,0]
arm_att_exp=[0,0,0]
cap_set =0 
good_posture=0
#########################
def main():
    global volBar,posx_flt,posy_flt,posz_flt,cap_flt,arm_pos_exp,arm_att_exp,cap_set,good_posture
    cap = cv2.VideoCapture(2)
    cTime = 0
    pTime = 0
    cap.set(3,wCam)
    cap.set(4,hCam)
    detector = handDetector1(detectionCon=0.8, maxHands=1)#one hand is fast
    # 接收创建贪吃蛇的类
    game = SnakeGameClass()
    print("Thread Hand Start!")
    while True:
        # Get image frame
        success, img = cap.read()
        img = cv2.flip(img,1)
        # Find the hand and its landmarks
        hands, img = detector.findHands(img,flipType=False)  # with draw
        # hands = detector.findHands(img, draw=False)  # without draw
    
        if hands:
            # Hand 1
            hand1 = hands[0]
            lmList1 = hand1["lmList"]  # List of 21 Landmark points
            bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
            type =  hand1["type"] 

            centerPoint1 = hand1['center']  # center of the hand cx,cy
            handType1 = hand1["type"]  # Handtype Left or Right
    
            fingers1 = detector.fingersUp(hand1)
            #print(fingers1)
            good_posture=0
            if fingers1[2]+fingers1[3]+fingers1[4]>=3 or 0:
               good_posture=1
            if len(hands) == 2:# unused
                # Hand 2
                hand2 = hands[1]
                lmList2 = hand2["lmList"]  # List of 21 Landmark points
                bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                centerPoint2 = hand2['center']  # center of the hand cx,cy
                handType2 = hand2["type"]  # Hand Type "Left" or "Right"
    
                fingers2 = detector.fingersUp(hand2)
                # Find Distance between two Landmarks. Could be same hand or different hands
                length, info, img = detector.findDistance(lmList1[8], lmList2[8], img)  # with draw
                #print("length is ",length)
                # length, info = detector.findDistance(lmList1[8], lmList2[8])  # with draw
            elif len(hands) == 1 and type == "Right": 
                bbox = hand1["bbox_lower"]  # Bounding box info x,y,w,h
                # 获取食指根部'5'和小指根部'17'的坐标点
                x1, y1 = lmList1[5] 
                x2, y2 = lmList1[17]
                # 勾股定理计算关键点'5'和'17'之间的距离，并变成整型
                distance = int(math.sqrt((x2-x1)**2 + (y2-y1)**2))
                #print('distance between 5 and 17:', distance)
                # 拟合的二次多项式的系数保存在coff数组中，即掌间距离和手与相机间的距离的对应关系的系数
                A, B, C = coff
                # 得到像素距离转为实际cm距离的公式 y = Ax^2 + Bx + C
                distanceCM = A*distance**2 + B*distance + C
                #print('distance CM:', distanceCM)

                size_hand = bbox[2]*bbox[3]/100
                hand_dis= distanceCM/100.#-size_hand/size_hand_base*size_to_dis+dis_hand_base
                # if size_hand:
                #     k_pose = size_hand_base/size_hand
                # else:
                #     k_pose = 1e9
                k_pose = hand_dis/dis_hand_base

                x1,y1 = lmList1[4][0],lmList1[4][1]# 0-> 1-Y
                x2,y2 = lmList1[8][0],lmList1[8][1]
                cx,cy = (x1+x2)//2,(y1+y2)//2# cap mid
                att=math.atan2(y2-y1, x2-x1)*180/math.pi
                #print(att)
                pix_x = (bbox[0]-wCam/2)*-1*k_pose
                pix_y = (bbox[1]-hCam/2)*-1*k_pose

                pose_x = pix_x*size_to_dis
                pose_y = pix_y*size_to_dis
                

                length = math.hypot(x2-x1,y2-y1)*k_pose
    
                vol = np.interp(length,[50,300],[minVol,maxVol])
                volBar = np.interp(length*2, [50, 300], [400, 150])
                volPer = np.interp(length*2, [50, 300], [0, 100])


 
                #----------------------filter-------------------------
                posx_flt=pose_x*flt_rate+(1-flt_rate)*posx_flt
                posy_flt=pose_y*flt_rate+(1-flt_rate)*posy_flt
                posz_flt=hand_dis*flt_rate+(1-flt_rate)*posz_flt
                cap_flt=length*flt_rate+(1-flt_rate)*cap_flt
                if 0:
                    print("hand_dis=",int(posz_flt*100),size_hand,k_pose)
                    print("cap pos=",int(posx_flt*100) ,int(posy_flt*100))
                    print("cap rate=",int(cap_flt))
                
                if posz_flt>0.3 and posz_flt<0.86 and  good_posture:#check for protect
                    # 更新贪吃蛇的节点，给出蛇头节点坐标。返回更新后的图像
                    pointIndex = [cx,cy]#lmList1[8][0:2]  # 只获取食指指尖关键点的（x,y）坐标

                    img = game.update(img, pointIndex)

                    cv2.rectangle(img, (5+50, int(-posx_flt*100/4)+240), (15+50, 240), (255, 255, 0), cv2.FILLED)
                    cv2.rectangle(img, (5+70, int(-posy_flt*100/4)+240), (15+70, 240), (0, 255, 0), cv2.FILLED)
                    cv2.rectangle(img, (5+90, int(-(posz_flt)*200)+240), (15+90, 240), (0, 0, 255), cv2.FILLED)
                    cv2.rectangle(img, (5+110, int(-(cap_flt)/2)+240), (15+110, 240), (255, 0, 255), cv2.FILLED)

                    cv2.circle(img,(x1,y1),15,(255,0,255),cv2.FILLED)
                    cv2.circle(img,(x2,y2),15,(255,0,255), cv2.FILLED)
                    cv2.line(img,(x1,y1),(x2,y2),(255,0,255),3)
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

                    str_pos ="["+str(int(posx_flt*100))+","+str(int(posy_flt*100))+","+str(int(posz_flt*100))+"]"
                    cv2.putText(img,str_pos,(bbox[0] - 30+30, bbox[1] - 80),cv2.FONT_HERSHEY_PLAIN,
                                                    2,(255, 255, 255),2)
                    
                    arm_pos_exp=[posx_flt,posy_flt,posz_flt]
                    arm_att_exp[0]=att
                    cap_set =cap_flt
                    if length<40:
                        cv2.circle(img, (cx, cy), 15, (0,255,0), cv2.FILLED)
                cx=100
                cy=300
                if fingers1[2]==1:
                    cv2.circle(img, (cx, cy), 15, (0,255,0), cv2.FILLED)
                else:
                    cv2.circle(img, (cx, cy), 15, (0,100,0), cv2.FILLED)

                cx=100
                cy=300+45
                if fingers1[3]==1:
                    cv2.circle(img, (cx, cy), 15, (0,255,0), cv2.FILLED)
                else:
                    cv2.circle(img, (cx, cy), 15, (0,100,0), cv2.FILLED)

                cx=100
                cy=300+45*2
                
                if fingers1[4]==1:
                    cv2.circle(img, (cx, cy), 15, (0,255,0), cv2.FILLED)
                else:
                    cv2.circle(img, (cx, cy), 15, (0,100,0), cv2.FILLED)
        # Display
        if 0:
            cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
            cv2.rectangle(img, (50, int(volBar)), (85, 400), (255, 0, 0), cv2.FILLED)
        size_x =0.4
        size_y =0.3
        img = cv2.resize(img,(0,0),fx=size_x,fy=size_y)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
 
def send_float(tx_Buf,data):
    temp_B= struct.pack('f',float(data))
    tx_Buf.append(temp_B[0])
    tx_Buf.append(temp_B[1])
    tx_Buf.append(temp_B[2])
    tx_Buf.append(temp_B[3])

def send_int(tx_Buf,data):
    temp_B= struct.pack('i',int(data))
    tx_Buf.append(temp_B[0])
    tx_Buf.append(temp_B[1])
    tx_Buf.append(temp_B[2])
    tx_Buf.append(temp_B[3])

def send_char(tx_Buf,data):
    tx_Buf.append(int(data))

def decode_float(rx_Buf,start_Byte_num):
    global rx_num_now
    temp=bytes([rx_Buf[start_Byte_num],rx_Buf[start_Byte_num+1],rx_Buf[start_Byte_num+2],rx_Buf[start_Byte_num+3]])
    rx_num_now= rx_num_now+4
    return struct.unpack('f',temp)[0]

def udp_convert_rx(data_rx): 
    global rx_num_now,start_replay
    tx_data_udp_temp=[]
    rx_num_now = 0
    start_replay=decode_float(data_rx,rx_num_now)

def udp_convert_tx_mode(): 
    global tx_data_udp,arm_pos_exp,arm_att_exp,cap_set,good_posture
    tx_data_udp_temp=[]
     
    send_float(tx_data_udp_temp, good_posture)
    send_float(tx_data_udp_temp,arm_pos_exp[0])
    send_float(tx_data_udp_temp,arm_pos_exp[1])
    send_float(tx_data_udp_temp,arm_pos_exp[2])
    send_float(tx_data_udp_temp,arm_att_exp[0])
    send_float(tx_data_udp_temp,arm_att_exp[1])
    send_float(tx_data_udp_temp,arm_att_exp[2])
    send_float(tx_data_udp_temp,cap_set)
 
    # print("cap pos=",int(arm_pos_exp[0]*100) ,int(arm_pos_exp[1]*100),int(arm_pos_exp[2]*100))
    # print("cap rate=",int(cap_set))
    for i in range(len(tx_data_udp_temp)):
        tx_data_udp[i]=tx_data_udp_temp[i]
    return len(tx_data_udp_temp) 

def thread_udp_send():#状态 客户端 
    global tx_data_udp
    addr_udp_ocu = ("127.0.0.1", 3333)#send to this port
    ser_udp_ocu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #local_addr = ("", 3334)#本地端口
    #ser_udp_ocu.bind(local_addr)
    print("UDP Thread start!!")
    delay_time=0.02
    while 1:
        time.sleep(delay_time)
        try:
            len_tx=udp_convert_tx_mode()#转换发送协议
        except:
            len_tx=0

        if len_tx:
            tx_data_temp =  [0]*len_tx
            for i in range(len_tx):
                tx_data_temp[i]=tx_data_udp[i]
            try:
                ser_udp_ocu.sendto(bytearray (tx_data_temp), addr_udp_ocu)#在此将机器人状态数据回传OCU
            except:
                print("Sending Error!!!\n")

            #print(tx_data_temp)
        #读取遥控器指令，并由状态机完成操控
        try:
            data, addr = ser_udp_ocu.recvfrom(256)
            if data:
                udp_convert_rx(data)
                #print ("got data from", addr, 'len=',len(data))#在此对遥控器下发指令进行解码
                #print (data)
        except:
            if data:
                udp_convert_rx(data)
            print ("err UDP data from", addr, 'len=',len(data))
 

if __name__ == "__main__":
    # 创建线程对象
    t1 = Thread(target=main)
    t2 = Thread(target=thread_udp_send)
    t1.start()
    t2.start()
    t1.join()     
    t2.join()
