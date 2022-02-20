
Fs = 5120;
t = 0:1/Fs:1;
y = sin(10*2*pi*t);
plot(abs(fft(y)))

