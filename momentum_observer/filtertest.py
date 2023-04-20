
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.fft import fft, ifft, fftfreq

from scipy import signal

from scipy.fft import fftshift
from numpy import genfromtxt
my_data = genfromtxt('../FT_data_horisontal.csv', delimiter=',')


#convert the csv file to numpy array

fx = my_data[:,0]
fy = my_data[:,1]
fz = my_data[:,2]
N = my_data.shape[0]
T = 1/500
xf = fftfreq(N, T)[:N//2]

#plot the force in x direction

#yf = fft(fx)

#sos = signal.butter(7, 10, 'hp', fs=500, output='sos')
#b, a = signal.butter(3, [1,2], 'band', fs=500, output='ba')




#plot the filtered force in x direction

import scipy.signal
from sklearn.metrics import mean_absolute_error as mae
from livefilter import LiveLFilter

#yraw = my_data[:,0][1:5660]
yraw = fx

fs = 500

# define lowpass filter with 2.5 Hz cutoff frequency
b, a = scipy.signal.iirfilter(10, Wn=50, fs=fs, btype="high", ftype="butter")
y_scipy_lfilter = scipy.signal.lfilter(b, a, yraw)

live_lfilterx = LiveLFilter(b, a)
live_lfiltery = LiveLFilter(b, a)
live_lfilterz = LiveLFilter(b, a)
# simulate live filter - passing values one by one
x_live_lfilter = [live_lfilterx._process(y) for y in fx]
y_live_lfilter = [live_lfiltery._process(y) for y in fy]
z_live_lfilter = [live_lfilterz._process(y) for y in fz]


xf = fftfreq(N, T)[:N//2]

yf = fft(y_live_lfilter)
yfo = fft(yraw)

#plt.plot(fft(yraw), label="raw")
#plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
#plt.plot(xf, 2.0/N * np.abs(yfo[0:N//2]))
#subplots

fig, axs = plt.subplots(3, 2)

axs[0, 0].plot(fx)
axs[0, 0].set_title('Raw Force in X')
axs[0, 1].plot(x_live_lfilter)
axs[0, 1].set_title('Filtered Force in X')
axs[1, 0].plot(fy)
axs[1, 0].set_title('Raw Force in Y')
axs[1, 1].plot(y_live_lfilter)
axs[1, 1].set_title('Filtered Force in Y')
axs[2, 0].plot(fz)
axs[2, 0].set_title('Raw Force in Z')
axs[2, 1].plot(z_live_lfilter)
axs[2, 1].set_title('Filtered Force in Z')



#plt.plot(y_live_lfilter)
#plt.plot(yraw)
#test = signal.medfilt(y_live_lfilter, 5)
#plt.plot(test)


#print(f"lfilter error: {mae(y_scipy_lfilter, y_live_lfilter):.5g}")
plt.show()



