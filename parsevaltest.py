import numpy as np

func=np.array([1,1,0,1])
sig=np.array([1,0,0,0,0,0,0,0,0])


z=np.convolve(sig,func[::-1],"valid")






#print(F1)
#print(F2)


