import numpy as np
from scipy.fftpack import fft,ifft
import scipy.io as sio              # 读取数据包
import matplotlib.pyplot as plt
import math

# data = sio.loadmat('/mnt/Documents/0.0-Adaptive_Oscillators/hip_flex_data.mat')
data = sio.loadmat('D:/0.0-Adaptive_Oscillators/hipdata.mat')
y = data['hipdata'][0]
Fs = 1/0.1
t = np.linspace(0,len(y)*0.1,len(y))
#Fs = 1000
#t = np.linspace(0,2,Fs*2)
#y = 60*np.cos(50*(2*np.pi)*t+np.pi/3)+72*np.cos(100*(2*np.pi)*t+np.pi/4)+50+72*np.cos(75*(2*np.pi)*t+np.pi/8)

# amplify
A_f = fft(y)
A_abs = abs(A_f)/y.size
A = np.hstack((A_abs[0],2*A_abs[1:int(y.size/2)]))

# frequent
f = Fs/2*np.linspace(0,1,int(y.size/2))
plt.plot(f,A)

# phase
y_real = np.real(A_f)
y_img = np.imag(A_f)
m = map(lambda x,y:math.atan2(y,x),y_real,y_img)
phase = np.array(list(m))[0:int(y.size/2)]
plt.plot(f,phase[0:int(y.size/2)]*180/np.pi)
plt.show()

# generate y input
y_gen = np.zeros((1,len(t)))[0]
for i in range(len(f)):
    y_gen = y_gen+A[i]*np.cos(f[i]*2*np.pi*t+phase[i])
plt.plot(t,y)
plt.plot(t,y_gen)
plt.show()

threshold = np.sum(A)/len(A)*2
print('f',f[A>threshold])
print('A',A[A>threshold])
print('phase',phase[A>threshold]/np.pi)
