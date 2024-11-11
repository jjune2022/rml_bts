import numpy as np

x= 1
y= 2
z = 3

a = np.array([[x], [y], [z]])
b = np.empty((3,0))
print(a)
print(b)

for i in range(10):
  b =  np.concatenate((a,b),axis=1)
  print(b)

print(np.linalg.norm(b[:,-1]-b[:,-2]) == 0.0)
print(np.linalg.norm(b[:,-1]-b[:,-2]) == 0)
c= b[:,1]
print(c)
print(c[1])
