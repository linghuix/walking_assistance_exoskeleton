
% 自相关函数：输入 N_t采样点数，X_t采样值，Fs 采样信号
function self = relax_self(X_t)
    N_t = length(X_t);
    % matlab的数组最小值从1开始，所以需要变化一下。
    Rxx = zeros(1,N_t);
    for r = 1:N_t
        for n = 1:N_t-r+1
            Rxx(r) = Rxx(r) + X_t(n)*X_t(n+r-1);
        end

        Rxx(r) = Rxx(r)/(N_t-r+1);
    end
    self = Rxx;
end