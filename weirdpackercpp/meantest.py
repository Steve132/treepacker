import numpy as np

N=100
A=np.random.rand(3,N)
#print(A)
mA=np.reshape(np.mean(A,1),(3,1))
#print(mA)
cA=A-mA
cQ=np.dot(cA,cA.T)
#print(cQ)
S=np.reshape(np.sum(A,1),(3,1))
Q=np.dot(A,A.T)-np.dot(S,S.T)/N
#print(Q)
A=np.array([[2.0/3.0,1.0/3.0],[2.0/3.0,1.0/3.0]])
print(A)

mA=np.reshape(np.mean(A,1),(2,1))
print(mA)
cQ=np.dot(A-mA,(A-mA).T)
print(cQ)
w,v=np.linalg.eig(cQ)
print((w,v))
