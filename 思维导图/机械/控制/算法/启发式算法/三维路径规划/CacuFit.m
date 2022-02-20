function fitness = CacuFit(path,a,b,h)
%%该闲数用于计笕路径适应度值 
% path		input	路径
% fitness	input	路径适应度值

[n,m] = size(path); 
% 对于每一只蚂蚁
for i = 1:n
	fitness(i) = 0; 
	for j = 2: m/2
		%适应度值为长度加高度，j*2-1为 y，j*2为 h
		fitness(i) = fitness(i) + sqrt((1*b)^2+((path(i,j*2-1)-path(i,(j-1)*2-1))*a)^2+((path(i,j*2)-path(i,(j-1)*2))*h)^2) + abs(path(i,j*2)*h);
	end
end