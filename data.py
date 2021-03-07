import numpy as np
import matplotlib.pyplot as plt



def loadData(filePath):
    fr = open(filePath, 'r+')
    lines = fr.readlines()
    length = len(lines)
    # 这里可以记录一下挨个数值分别是啥意思
    ep_0 = []
    ep_1 = []
    ep_2 = []
    for line in lines:
        items = line.strip().split('\t')
        # print(items)
        ep_0.append(float(items[0]))
        ep_1.append(float(items[1]))
        ep_2.append(float(items[2]))
    return ep_0, ep_1, ep_2, length


if __name__ == '__main__':
    ep_0, ep_1, ep_2, length = loadData('data.txt')
    x = list(range(1, length+1))

#? af
#knlknk
plt.xlabel('X')
plt.ylabel('Y')
plt.title('trace')

plt.plot(ep_0, ep_1)
#plt.plot(x, ep_1, linestyle='--')
# plt.plot(x, ep_2, linestyle=':')

plt.legend(('X_trace', 'ep_1', 'ep_2'), loc='upper right')

plt.show()


