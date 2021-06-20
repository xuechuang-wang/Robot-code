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
    # theta = []
    vc = []
    wc = []
    vE = []
    wE = []

    xR = []
    yR = []
    # thetaR = []
    xE = []
    yE = []

    rou1_guji =[]
    rou2_guji =[]


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

        rou1_guji.append(float(items[10]))
        rou2_guji.append(float(items[11]))



    return x, y, xR, yR,vc,wc,vE,wE, xE, yE,rou1_guji,rou2_guji, length


if __name__ == '__main__':
    # x, y, xR, yR,vc,wc,vE,wE, xE, yE,rou1_guji,rou2_guji ,length = loadData('./build/bin/data.txt')
    x, y, xR, yR,vc,wc,vE,wE, xE, yE,rou1_guji,rou2_guji ,length = loadData('./data.txt')
    t = list(range(1, length+1))


plt.xlabel('X')
plt.ylabel('Y')
plt.title('trace')



plt.figure(1)

plt.plot(xR, yR, linestyle='--')
plt.plot(x, y)
plt.legend(('reference', 'real'), loc='upper right')



plt.figure(2)

plt.plot(xE[0:2000], linestyle='--')
plt.plot(yE[0:2000])
plt.legend(('x-xr', 'y-yr'), loc='upper right')


plt.figure(3)
plt.plot(vE[0:2000])
plt.plot(wE[0:2000])
plt.legend(('v-vr','w-wr'),loc = 'upper right')

plt.figure(4)
plt.plot(vc[0:2000])
plt.plot(wc[0:2000])
plt.legend(('vc','wc'),loc = 'upper right')



plt.figure(5)
plt.plot(rou1_guji[0:2000])
plt.plot(rou2_guji[0:2000])
plt.legend(('rou1_guji','rou2_guji'),loc = 'upper right')


plt.show()


