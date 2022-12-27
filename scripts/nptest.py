import numpy as np 

features1 = np.array([[1,2,3],[11,12,13],[21,22,23]])
features2 = np.array([[4,5,6],[14,15,16],[24,25,26]])
tempf1 = features1[1,:] * np.ones(features2.shape)
print (tempf1)