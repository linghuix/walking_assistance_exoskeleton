function m = sine_coff(data,nn)
    % nn——反映三角函数系数个数。
        % length(data)为偶数时，最大可以取length(data)/2+1
        % length(data)为奇数时，最大可以取length(data)%2+1
    % data——需要返回三角函数展开系数的数据集
    clc

%     x = 1:50;
%     q =[ 0.1839 0.2400 0.4173 0.0497]
%     data = (q(1)*cos(q(2).*x)+q(3)*cos(q(4).*x))';
%     nn = 26;


    le = length(data);
    con = fft(data);
    
%     plot(ifft(con),'-r')
%     hold on
    
%% 傅里叶级数展开
    a = con;
    n = a(2:nn);
    % a(1)为常数项
    % 前半部real为cos(2*pi/le*i*x)
    % 后半部imag为sin(2*pi/le*i*x)系数
    m = [a(1);2*real(n);-2*imag(n)]; 
    % output nn*1
    
%     q=[];
%     for k = (0:le-1)
%         q = [q f(k,m,le)];
%     end
%     plot(q,'y')
end


    