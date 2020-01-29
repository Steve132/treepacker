import numpy as np

func=np.array([1,1,0,1,0,0,0,0,0,0,0,0])
sig= np.array([1,0,0,0,0,0,0,0,0,0,0,0])


Ffunc=np.fft.fft(func,n=12,norm="ortho")
Fsig=np.fft.fft(sig,n=12,norm="ortho")
Fsig_1=np.fft.fft([0,1,0,0,0,0,0,0,0,0,0,0],n=12,norm='ortho')
shift1=Fsig_1/Fsig

#print(Fsig)
#print(Fsig_1)
#print(Fsig_1/Fsig)

#print(np.linalg.norm(func+sig,ord=1))
#print(np.linalg.norm(Ffunc+Fsig,ord=np.inf))

for k1 in range(10):
	for k2 in range(12): 
		Fy=Ffunc*(shift1**k1)+Fsig+Ffunc*(shift1**k2)
		print((k1,k2,np.linalg.norm(Fy,ord=2)))
