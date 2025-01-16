import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('datalog.txt')


X = data[:,3]
X2 = data[:,12]
T = len(X)



plt.plot(np.arange(0, T*0.004, 0.004), X)


plt.title('Data Plot')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)


plt.show()