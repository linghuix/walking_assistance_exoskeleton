function result = f(x, coff, len)
    % x，自变量
    % coff, function
    % sine_off的输出。a(1)为常数项，前半部real为cos(2*pi/le*i*x)，后半部imag为sin(2*pi/le*i*x)系数。
            %注，系数并没有除以采样点数。但是是乘以了二的
    % len，总体采样点的个数，
    % result 返回一定频率下(coff)因变量的函数值
    
    
    % mean，均值
    % cos_cof cos函数的系数
    % sin_cof sin函数的系数
    mean = coff(1)/len;                            %constant
    cos_cof = coff(2:(length(coff)+1)/2)/len;      %cos(2*pi/le*i*x) i from 1 to (le-1)/2
    sin_cof = coff((length(coff)+1)/2+1:end)/len;  %sin(2*pi/le*i*x) i from 1 to (le-1)/2
    
    y = 0;

    for index = 1:length(cos_cof)
        y = y + cos_cof(index)*cos(2*pi/len*index*x);
    end

    for index = 1:length(sin_cof)
        y = y + sin_cof(index)*sin(2*pi/len*index*x);
    end    
    length(sin_cof)
    result = mean + y;
end
