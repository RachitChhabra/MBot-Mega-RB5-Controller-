import numpy as np

from numpy import genfromtxt
my_data = genfromtxt("/data/sig.csv", delimiter=',')
my_data = np.load("/data/sig.npy")
print(my_data)

# wp1 = np.argmin(data[:,0:2] - [0,0])
