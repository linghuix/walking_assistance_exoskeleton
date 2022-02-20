 function [Y] = AF_foodconsistence(X)
%计算人工鱼的当前位进的食物浓度 

%输入：
%X	待求的人工鱼，每列为一条人工鱼

%输出：
%Y	输出各条人工鱼当前位置的食物浓度（即闲数值）

fishnum = size(X,2);
for i = 1:fishnum
% 	Y(1,i) = X(i)*sin(10*pi*X(i))+2;
Y(1,i) = 1/(X(i)+2)*sin(50*pi*X(i))+2
end
