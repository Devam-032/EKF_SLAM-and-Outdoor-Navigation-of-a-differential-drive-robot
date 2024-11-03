import numpy as np
a=np.array([[0,0],[0,0]]*10)
for i in range(0,len(a)):
    if(i%2==0):
        a[i]=1
print(a)