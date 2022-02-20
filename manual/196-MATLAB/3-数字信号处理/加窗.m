
length = 50;
t = (1:length) + 50 
y = sin(5*pi/200*t)';


subplot(3,2,1)
plot(y)
xlabel('signal')
subplot(3,2,2)
plot(abs(fft(y)))
xlabel('signal')


subplot(3,2,3)
window = chebwin(length)
% window = hanning(length);
plot(window);
xlabel('hanning windows')
subplot(3,2,4)
plot(abs(fft(window)))
xlabel('signal')


subplot(3,2,5)
y_w = y.*hanning(length)
plot(y_w);
xlabel('signal X hanning windows')
subplot(3,2,6)
plot(abs(fft(y_w)))
xlabel('signal')

