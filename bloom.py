import sys
import math

n=(8*128)**2
m=int(sys.argv[1])
k=(m/n)*math.log(2.0)

err=(1.0-math.exp(-k*n/m))**k

print(err)
