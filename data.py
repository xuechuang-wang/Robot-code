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
    xE = []
    yE = []

    for line in lines:
        items = line.strip().split(',')
        # print(items)
        x.append(float(items[0]))
        y.append(float(items[1]))
        theta.append(float(items[2]))
        xR.append(float(items[3]))
        yR.append(float(items[4]))
        thetaR.append(float(items[5]))

        xE.append(float(items[-2]))
        yE.append(float(items[-1]))


    return x, y, theta, xR, yR, thetaR, xE, yE, length


if __name__ == '__main__':
    x, y, theta, xR, yR, thetaR0, xE, yE, length = loadData('data.txt')
    t = list(range(1, length+1))


plt.xlabel('X')
plt.ylabel('Y')
plt.title('trace')



plt.figure(1)

plt.plot(xR, linestyle='--')
plt.plot(x)
plt.legend(('xR', 'x'), loc='upper right')



plt.figure(2)

plt.plot(yR, linestyle='--')
plt.plot(y)
plt.legend(('yR', 'y'), loc='upper right')


plt.figure(3)
plt.plot(x)
plt.plot(xE)
plt.legend(('x','xE'),loc = 'upper right')

plt.figure(4)
plt.plot(y)
plt.plot(yE)
plt.legend(('y','yE'),loc = 'upper right')






plt.show()


