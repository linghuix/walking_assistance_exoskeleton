% 离散傅里叶逆变换：输入 N_t采样点数，X_t采样值，Fs 采样信号
%   The inverse DFT (computed by IFFT) is given by
%                    N
%      x(n) = (1/N) sum  X(k)*exp( j*2*pi*(k-1)*(n-1)/N), 1 <= n <= N.
%                   k=1
function X_t = my_iDFT(freq)
    N_t = length(freq);
    % matlab的数组最小值从1开始，所以需要变化一下。
    W = exp(-j*2*pi/N_t);
    X = zeros(1,N_t);
    for k = 1:N_t
        for n = 1:N_t
            X(k) = X(k) + freq(n)*W^(-(k-1)*(n-1));
        end

        X(k) = X(k)/N_t;
    end
    X_t=real(X);%  取实部将复数转化为实数（5+000j --> 5）
end