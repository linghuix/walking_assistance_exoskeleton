% fit data
x0 = -20:5:35;
y=[0 0 100 800 2000 1800 1300 800 500 100 0 0];

% fit 1
x = -20:.5:35;
a1 =1398; b1=0.5224; c1=5.811; a2=1179; b2=8.902; c2=10.94;
f =  a1*exp(-((x-b1)/c1).^2) + a2*exp(-((x-b2)/c2).^2)-1.1028;

plot(x,f)
hold on
plot(x0,y,'o')
%% 精确通过（-20,0）；（35,0）

% fit 2
a1=-2.284;b1=4.72;c1=-12.58;
f = (a1*exp(-((x-b1)/c1).^2) ).*(x+20).*(x-35);

plot(x,f)
hold on
plot(x0,y,'o')
	%Goodness of fit: SSE: 2.03e+05 R-square: 0.9657 Adjusted R-square: 0.9581 RMSE: 150.2
%% 精确通过（-20,0）；（35,0）

% fit 3
a1=3.464;a2=-3.534;b1=-11.03;b2=-6.575;c1=-8.057;c2=20.27;
f = (a1*exp(-((x-b1)/c1).^2) + a2*exp(-((x-b2)/c2).^2)).*(x+20).*(x-35)

plot(x,f)
hold on
plot(x0,y,'ro')
	%Goodness of fit:SSE: 7.917e+04 R-square: 0.9866 Adjusted R-square: 0.9755 RMSE: 114.9