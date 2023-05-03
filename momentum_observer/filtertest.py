
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.fft import fft, ifft, fftfreq

from scipy import signal

from scipy.fft import fftshift
from numpy import genfromtxt
my_data = genfromtxt('../FT_data_log6.csv', delimiter=',')


#convert the csv file to numpy array


fs = 500
#"192.168.1.111"

b, a = signal.iirfilter(4, Wn=10, fs=fs, btype="low", ftype="butter")
c, d = signal.iirfilter(6, Wn=5, fs=fs, btype="low", ftype="butter")


#live_lfilterfx = LiveLFilter(b, a)




fx = my_data[:,6]
fy = my_data[:,7]
fz = my_data[:,8]

filteredx = my_data[:,0]
filteredy = my_data[:,1]
filteredz = my_data[:,2]


N = my_data.shape[0]
T = 1/500
xf = fftfreq(N, T)[:N//2]

#plot the force in x direction

#yf = fft(fx)

#sos = signal.butter(7, 10, 'hp', fs=500, output='sos')
#b, a = signal.butter(20, 5, 'low', fs=500, output='ba')




#plot the filtered force in x direction

import scipy.signal
from sklearn.metrics import mean_absolute_error as mae
from livefilter import LiveLFilter

#yraw = my_data[:,0][1:5660]
yraw = fx

fs = 500

#b, a = signal.iirfilter(4, 10, 1, 60, analog=True, ftype='cheby1')
#w, h = signal.freqs(b, a, worN=np.logspace(-1, 2, 1000))


#y = signal.filtfilt(b, a, yraw, padlen=50)
#plt.semilogx(w, 20 * np.log10(abs(h)))

#plt.xlabel('Frequency')

#plt.ylabel('Amplitude response [dB]')

#plt.grid(True)

#plt.show()
#plt.plot(y)
#plt.plot(yraw)

#
live_lfilterx = LiveLFilter(c, d)
live_lfiltery = LiveLFilter(b, a)
live_lfilterz = LiveLFilter(b, a)
## simulate live filter - passing values one by one
x_live_lfilter = [live_lfilterx._process(y) for y in fx]
y_live_lfilter = [live_lfiltery._process(y) for y in fy]
z_live_lfilter = [live_lfilterz._process(y) for y in fz]

#apply filter with filtfil

xtest = signal.filtfilt(b, a, fx, padlen=50) 

#
#xf = fftfreq(N, T)[:N//2]
#
#yf = fft(y_live_lfilter)
#yfo = fft(yraw)

#plt.plot(fft(yraw), label="raw")
#plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
#plt.plot(xf, 2.0/N * np.abs(yfo[0:N//2]))
#subplots

fig, axs = plt.subplots(3, 2)

axs[0, 0].plot(fx)
axs[0, 0].set_title('Raw Force in X')
axs[0, 1].plot(filteredx)
axs[0, 1].set_title('Filtered Force in X')
axs[1, 0].plot(fy)
axs[1, 0].set_title('Raw Force in Y')
axs[1, 1].plot(filteredy)
axs[1, 1].set_title('Filtered Force in Y')
axs[2, 0].plot(fz)
axs[2, 0].set_title('Raw Force in Z')
axs[2, 1].plot(filteredz)
axs[2, 1].set_title('Filtered Force in Z')



#plt.plot(y_live_lfilter)
#plt.plot(yraw)
#test = signal.medfilt(y_live_lfilter, 5)
#plt.plot(test)


#print(f"lfilter error: {mae(y_scipy_lfilter, y_live_lfilter):.5g}")
plt.show()



