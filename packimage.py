import numpy as np
from imageio import imread,imwrite
import matplotlib.pyplot as plot
import scipy,scipy.signal
from time import perf_counter 

def findcandidatepositions(mask1,mask2):
	Result=scipy.signal.convolve(mask1,mask2,"valid")
	Result=Result[mask2.shape[0]:,mask2.shape[1]:]
	return np.argwhere(Result == 0)

def prepare_mask(Mask):
	Mask=np.greater(Im[:,:,-1],0).astype(dtype=np.int32)
	for d in range(Mask.ndim):
		Mask=np.flip(Mask,axis=d)
	Mask=np.copy(Mask)
	return Mask

def embed_images(part_list,current_image,used_indices=frozenset()):		#list of lists of part configurations
	if(len(part_list)==len(used_indices)):
		return True

	for pi,p in enumerate(part_list):
		if(pi not in used_indices):
			current_image=np.copy(current_image)
			for ci,c in enumerate(p):
				



Im=imread('data/examplepart.png')

A=np.zeros((2000,2000),dtype=np.int32)
b=perf_counter()
P=findcandidatepositions(A,Mask)
print(perf_counter()-b)
#B=np.ones((25,25))

#C=convn_full(A,B)
#print(F1)
#print(F2)
#func=np.array([1,1,0,1])
#sig=np.array([1,0,0,0,0,0,0,0,0])

#z=scipy.signal.convolve(sig,func[::-1],"valid")


