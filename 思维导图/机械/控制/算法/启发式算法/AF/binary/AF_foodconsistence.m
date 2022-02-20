 function [Y] = AF_foodconsistence_2(X)
%计算人工鱼的当前位进的食物浓度 

%输入：
%X	待求的人工鱼，每列为一条人工鱼

%输出：
%Y	输出各条人工鱼当前位置的食物浓度（即闲数值）

fishnum = size(X,2);
for i = 1:fishnum
	X1 = X(1,i);
	X2 = X(2,i);
	Y(1,i) = sin(X1)*sin(X2)/X1/X2;
end
