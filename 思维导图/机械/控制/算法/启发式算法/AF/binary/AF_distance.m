 function D = AF_distance(Xi,X)
%计箅第i条鱼与所有鱼的位置

%输入:
%Xi 第i条鱼的当前位置 
%X	所有鱼的当前位罝

%输出:
%D 第i条鱼与所有鱼的距离 

col = size(X,2);
D = zeros(1,col); 
for j = 1:col
	D(j) = norm(Xi-X(:,j)); 
end 