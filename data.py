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
    theta = []

    xR = []
    yR = []
    thetaR = []
    x1guji = []
    xE = []

    for line in lines:
        items = line.strip().split(',')
        # print(items)
        x.append(float(items[0]))
        y.append(float(items[1]))
        theta.append(float(items[2]))
        xR.append(float(items[3]))
        yR.append(float(items[4]))
        thetaR.append(float(items[5]))

        x1guji.append(float(items[-4]))
        xE.append(float(items[-3]))


    return x, y, theta, xR, yR, thetaR, x1guji,xE, length


if __name__ == '__main__':
    x, y, theta, xR, yR, thetaR, x1guji, xE, length = loadData('data.txt')
    t = list(range(1, length+1))


plt.xlabel('X')
plt.ylabel('Y')
plt.title('trace')



plt.plot(xE, linestyle='--')
#plt.plot(x, ep_1, linestyle='--')
# plt.plot(x, ep_2, linestyle=':')
plt.plot(x1guji)
plt.legend(('xE', 'guji'), loc='upper right')

plt.show()


