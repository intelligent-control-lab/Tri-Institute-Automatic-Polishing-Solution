import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('datalog.txt')


X = data[:,10]
X2 = data[:,11]
X3 = data[:,12]
X4 = data[:,16]
X5 = data[:,17]
X6 = data[:,18]
T = len(X)


plt.plot(np.arange(0, T*0.004, 0.004), X, label = '1')
plt.plot(np.arange(0, T*0.004, 0.004), X2, label = '2')
plt.plot(np.arange(0, T*0.004, 0.004), X3, label = '3')
plt.plot(np.arange(0, T*0.004, 0.004), X4, label = '4')
plt.plot(np.arange(0, T*0.004, 0.004), X5, label = '5')
plt.plot(np.arange(0, T*0.004, 0.004), X6, label = '6')

plt.title('Data Plot')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)


plt.show()