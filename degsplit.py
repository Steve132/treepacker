def splitdominantbisect(N):
	increment=N//2
	position=0
	order={}
	while(increment > 0):
		for k in range(0,N,increment):
			if(k not in order):
				order[k]=position
				position+=1
		increment = increment//2
	output=[0]*len(order)
	for k,v in order.items():
		output[v]=k
	return output
	



#print(F1)
#print(F2)


