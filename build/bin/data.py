# -*- coding: utf-8 -*-

from os import WEXITED
from matplotlib import lines
import numpy as np
import matplotlib.pyplot as plt



def loadData(filePath):
    fr = open(filePath, 'r+')
    lines = fr.readlines()
    length = len(lines)
    # 这里可以记录一下挨个数值分别是啥意思
    x = []
    y = []

    xR = []
    yR = []

    xE = []
    yE = []
    # theta = []
    vc = []
    wc = []

    vE = []
    wE = []

    thetaR = []
    thetaReal = []
    # thetaR = []
    rou1_guji =[]
    rou2_guji =[]

    xHat = []
    yHat = []


    for line in lines:
        items = line.strip().split(',')
        # print(items)
        x.append(float(items[0]))
        y.append(float(items[1]))
        # theta.append(float(items[2])) x, y, xr, yr, xe, ye, vr, wr, ve, we
        
        xR.append(float(items[2]))
        yR.append(float(items[3]))
        # thetaR.append(float(items[5]))

        xE.append(float(items[4]))
        yE.append(float(items[5]))

        vc.append(float(items[6]))
        wc.append(float(items[7]))

        vE.append(float(items[8]))
        wE.append(float(items[9]))

        thetaR.append(float(items[10]))
        thetaReal.append(float(items[11]))

        rou1_guji.append(float(items[12]))
        rou2_guji.append(float(items[13]))

        xHat.append(float(items[14]))
        yHat.append(float(items[15]))


    return x, y, xR, yR,xE, yE, vc,wc,vE,wE, thetaR,thetaReal, rou1_guji,rou2_guji, xHat, yHat, length


if __name__ == '__main__':
    # x, y, xR, yR,vc,wc,vE,wE, xE, yE,rou1_guji,rou2_guji ,length = loadData('./build/bin/data.txt')
    x, y, xR, yR, xE, yE, vc, wc,vE,wE, thetaR, thetaReal, rou1_guji,rou2_guji , xHat , yHat,length = loadData('./data.txt')
    t = list(range(1, length+1))


plt.xlabel('X')
plt.ylabel('Y')
plt.title('trace')


plt.figure(1)
plt.plot(xR, yR, linestyle='--')
plt.plot(x, y)
plt.legend(('reference', 'real'), loc='upper right')



plt.figure(2)
plt.plot(xE[0:2000])
plt.grid(True)     # 加网格
plt.legend(('xr-x'), loc='upper right')

plt.figure(3)
plt.plot(yE[0:2000])
plt.grid(True)     # 加网格
plt.legend(('yr-y'), loc='upper right')

# plt.figure(4)
# plt.plot(vE[0:3000])
# plt.plot(wE[0:3000])
# plt.legend(('vc','wc'),loc = 'upper right')

plt.figure(5)
plt.plot(thetaR[0:3000])
plt.plot(thetaReal[0:3000])
plt.legend(('thetaR','thetaReal'),loc = 'upper right')


# plt.figure(5)
# plt.plot(rou1_guji[0:3000])
# plt.plot(rou2_guji[0:3000])
# plt.legend(('rou1_guji','rou2_guji'),loc = 'upper right')

# plt.figure(6)
# plt.plot(xHat[0:3000])
# plt.plot(yHat[0:3000])
# plt.legend(('xHat','yHat'),loc = 'upper right')


plt.show()


