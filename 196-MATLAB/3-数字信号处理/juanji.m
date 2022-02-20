function [y ny] = juanji(x,nx,h,nh)
% x为卷积序列，nx为序列的位置,是一维数组
% h为卷积序列，nh为序列的位置,是一维数组
ny_start = nh(1)+nx(1);
ny_end = nh(end)+nx(end);
y = conv(x,h);
ny = ny_start:ny_end;
end

% 例程

