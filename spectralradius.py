import numpy as np

A=np.random.randn(2,2)
A=np.eye(2,2)*3.1

print(np.linalg.eigvals(A))
print(np.linalg.norm(np.dot(A,A),2))
