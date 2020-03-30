from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port)
from pypozyx.tools.version_check import perform_latest_version_check
import numpy as np
import math

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import CheckButtons
import socket

class ReadyToRange(object):
    def __init__(self, pozyx, destination_id, range_step_mm=1000, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.protocol = protocol
    def setup(self):
        # set the ranging protocol
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)
    def loop(self):
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id)
        dis = device_range.distance
        return dis
'''
----------------------TOA-------------------------------
'''
def toafunc(H, r):
    # H
    dummy = np.zeros(H.shape)
    dummy = dummy+H[0]
    toaH = H-dummy
    # K**2
    toaK2 = (sum((H*H).T)).reshape(H.shape[0], 1)
    # r
    toar = r
    toar2 = (sum((toar*toar).T)).reshape(toar.shape[0], 1)
    # b
    toaB = 1/2*(toaK2-toaK2[0]-toar2+toar[0]**2)
    # row delete
    toaH = toaH[1:, :]
    toaB = toaB[1:, :]
    # toa prediction
    TOApred = np.linalg.inv(toaH.T@toaH)@toaH.T@toaB
    return TOApred
'''
---------------------Kalman-----------------------------
'''
def TrackKalman(z):
    global A, Hk, Q, R, xk, P
    # kalman
    xp = A@xk
    Pp = A@P@A.T+Q
    K = Pp@Hk.T@np.linalg.inv(Hk@Pp@Hk.T+R)
    xk = xp+K@(z-Hk@xp)
    P = Pp-K@Hk@Pp
    #
    point = np.array([[xk[0]], [xk[2]]])
    return point
'''
-------------------Kalman setting----------------------
'''
A = np.array([[1, 0.25, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.25], [0, 0, 0, 1]])
Hk = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
# 시스템 잡음
Q = np.eye(4)/200
# 측정 잡음
R = 0.25*np.eye(2)
xk = np.array([[0], [0], [0], [0]])
P = 100*np.eye(4)
'''
-------------------TOA setting------------------------
'''
# anchor address
anchor_array = (0x675f, 0x671b, 0x6758, 0x6714,
                0x6a5d, 0x6a5e, 0x672d, 0x6e06, 0x6e3c, 0x6917, 0x6e6e, 0x6a32)
# anchor location
H_array = ((2130,0),(7540,0),(14140,0),(14140,7580),(14140,15470),(7540,15360),
           (-250,15180),(0,7560),(7540,7210),(8140,7550),(7540,8210),(7130,7550))
# anchor room
H_room_AP = ((6,7,8,12),(1,2,8,12),(4,5,6,10),(2,3,4,10))

# tag address
#remote_id = [0x674f]
remote_id = (0x674f,0x673b,0x677d,0x6739,0x6728,0x672c,0x675a)
#remote_id = (0x674f,0x673b)

tag=len(remote_id)

#tag room initial setting
room_number=[0]*tag
# (tag to anchor height)^2
tag_height = [650,600,800,600,600,600,650]
tag_height = tuple([(2000-i)**2 for i in tag_height])

# TOA inital setting
TOApred = [[3000,10000]]*tag
# database
Track_data = []
trajectory=0

if __name__ == "__main__":
    '''
    -------------------Pozyx setup-----------------
    '''
    check_pypozyx_version = False
    if check_pypozyx_version:
        perform_latest_version_check()
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    range_step_mm = 1000
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
    pozyx = PozyxSerial(serial_port)
    ranging = [[ReadyToRange(pozyx, anchor_array[i], range_step_mm,
                             ranging_protocol, remote_id[j]) for i in range(0, len(anchor_array))] for j in range(0,tag)]
    ranging[0][0].setup()
    '''
    -------------------MAIN-----------------
    '''

    img = mpimg.imread('/Users/jaebok/Desktop/expe.png')
    fig, ax = plt.subplots()


    while True:
        # 측정한 tag 좌표를 기준으로 Room 배정
        for i in range(0,tag):
            if TOApred[i][0] < 7540:
                if 7310 < TOApred[i][1]:
                    room_number[i] = 0
                elif TOApred[i][1] <= 7310:
                    room_number[i] = 1
            elif TOApred[i][0] >= 7540:
                if 7310 < TOApred[i][1]:
                    room_number[i] = 2
                elif TOApred[i][1] <= 7310:
                    room_number[i] = 3

        # room에 맞는 anchor를 H에 넣는 작업
        H = [[H_array[i-1] for i in H_room_AP[room_number[j]]] for j in range(0,tag)]

        # 각 tag 당 ranging 값이 3개 이상 나오도록 설정
        while True:
            # ranging 실패한 앵커 갯수 카운터
            counter=[]
            #ranging
            r = [[ranging[j][i-1].loop() for i in H_room_AP[room_number[j]]] for j in range(0,tag)]

            # tag 중 하나라도 2개 이상으로 에러가 나면 restart
            # 1개만 에러가 나면 그 부분만 지우고 나머지 3개로 삼변측량
            # 0개면 그냥 앞에 3개만 써서 삼변측량
            error_tag=[]
            pass_tag=[]
            for i in range(0,tag):
                if r[i].count(0) >= 2:
                    error_tag.append(i)             
                elif r[i].count(0) == 1:
                    pass_tag.append(i)
                    del(H[i][r[i].index(0)])
                    r[i].remove(0)
                else:
                    pass_tag.append(i)
            break

        # z값도 고려해 distance 정확도 향상
        #r = [[math.sqrt(r[j][i]**2-tag_height[j]) for i in range(0,len(r[j]))] for j in pass_tag]
        for i in pass_tag:
            for j in range(0,len(r[i])):
                if r[i][j]**2-tag_height[i]>0:
                    r[i][j]=math.sqrt(r[i][j]**2-tag_height[i])
                else:
                    room_number[i]=0
                    TOApred[i][0]=3000      
                    TOApred[i][1]=8000                
        for i in error_tag:
            room_number[i]=0
            TOApred[i][0]=3000      
            TOApred[i][1]=8000
        # 삼변측량
        TOApred=np.array(TOApred)
        for i in pass_tag:
            TOApred[i] = toafunc(np.array(H[i][0:3]), np.array(r[i][0:3]).reshape(3, 1)).reshape(2)

        # Kalmanpred=TrackKalman(TOApred).reshape(1,2)
        #print(TOApred)

        # 칼만 결과가 array로 나와 list로 바꿔줌
        TOApred=TOApred.tolist()
        a=np.zeros((tag,3))
        for i in range(0,tag):
            a[i,:]=[room_number[i]]+TOApred[i]
        #trajectory+=1
        #print(trajectory)
        print(a)
        ax.clear()
        l0, = ax.plot(a[:,1]*0.10477+310, a[:,2]*-0.05825+1250,'o', lw=2, color='k', label='Ture')
        #l1, = ax.plot(a[:,1], a[:,2],'o', lw=2, color='r', label='Mult')
        #l2, = ax.plot(a[:,1], a[:,2],'o', lw=2, color='g', label='Kalman')

        plt.imshow(img)
        plt.xticks([]) # x축 눈금
        plt.yticks([]) # y축 눈금
        #plt.axis([0, 2000,0,2000])
        plt.text((a[0,1]*0.10477+310)/2044, 1-(a[0,2]*-0.05825+1250)/1452-0.02, 'tag 1', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[1,1]*0.10477+310)/2044, 1-(a[1,2]*-0.05825+1250)/1452-0.02, 'tag 2', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[2,1]*0.10477+310)/2044, 1-(a[2,2]*-0.05825+1250)/1452-0.02, 'tag 3', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[3,1]*0.10477+310)/2044, 1-(a[3,2]*-0.05825+1250)/1452-0.02, 'tag 4', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[4,1]*0.10477+310)/2044, 1-(a[4,2]*-0.05825+1250)/1452-0.02, 'tag 5', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[5,1]*0.10477+310)/2044, 1-(a[5,2]*-0.05825+1250)/1452-0.02, 'tag 6', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.text((a[6,1]*0.10477+310)/2044, 1-(a[6,2]*-0.05825+1250)/1452-0.02, 'tag 7', horizontalalignment='center',verticalalignment='center', transform=ax.transAxes)
        plt.pause(0.01)

# import pandas as pd
# data=pd.DataFrame(Track_data)
# data.to_csv('trackdata.csv')