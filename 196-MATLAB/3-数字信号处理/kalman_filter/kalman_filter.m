function [x, V, VV, loglik] = kalman_filter(y, A, C, Q, R, init_x, init_V, varargin)
% Kalman filter.搜索
% [x, V, VV, loglik] = kalman_filter(y, A, C, Q, R, init_x, init_V, ...)
%
% INPUTS:
% y(:,t)   - the observation at time t              在时间t的观测
% A - the system matrix                       A 系统矩阵         
% C - the observation matrix                  C 观测矩阵
% Q - the system covariance                   Q  系统协方差
% R - the observation covariance              R  观测协方差 
% init_x - the initial state (column) vector      init_x   初始状态（列）向量
% init_V - the initial state covariance             init_V    初始状态协方差
%
% OPTIONAL INPUTS (string/value pairs [default in brackets])             选择性输入（字符串/值 对【默认在括号中】）
% 'model' - model(t)=m means use params from model m at time t [ones(1,T) ]         在时间t，m意味着利用 m模型参数   [ones(1,T) ]
% 
%     In this case, all the above matrices take an additional final
%     dimension,                                                           在这种情况下,上述矩阵采用附加的维数
%     i.e., A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m).                     例如
%     However, init_x and init_V are independent of model(1).      
%                                                 init_x and init_V相对于模型1是独立的
% 'u'     - u(:,t) the control signal at time t [ [] ]
%                                                           在时间t的控制信号
% 'B'     - B(:,:,m) the input regression matrix for model m
%                                                         对于模型m的，输入回归矩阵

% OUTPUTS (where X is the hidden state being estimated)       输出（其中X是被估计的隐藏状态）
% x(:,t) = E[X(:,t) | y(:,1:t)]
% V(:,:,t) = Cov[X(:,t) | y(:,1:t)]
% VV(:,:,t) = Cov[X(:,t), X(:,t-1) | y(:,1:t)] t >= 2
% loglik = sum{t=1}^T log P(y(:,t))
%
% If an input signal is specified, we also condition on it:  如果一个输入信号是特定的,我们的条件
% e.g., x(:,t) = E[X(:,t) | y(:,1:t), u(:, 1:t)]
% If a model sequence is specified, we also condition on it:
% e.g., x(:,t) = E[X(:,t) | y(:,1:t), u(:, 1:t), m(1:t)]
[os T] = size(y);
ss = size(A,1); % size of state space
% set default params
model = ones(1,T);
u = [];
B = [];
ndx = [];
args = varargin;
nargs = length(args);
for i=1:2:nargs
  switch args{i}
   case 'model', model = args{i+1};
   case 'u', u = args{i+1};
   case 'B', B = args{i+1};
   case 'ndx', ndx = args{i+1};
   otherwise, error(['unrecognized argument ' args{i}])
  end
end
x = zeros(ss, T);
V = zeros(ss, ss, T);
VV = zeros(ss, ss, T);
loglik = 0;
for t=1:T
  m = model(t);
  if t==1
    %prevx = init_x(:,m);
    %prevV = init_V(:,:,m);
    prevx = init_x;
    prevV = init_V;
    initial = 1;
  else
    prevx = x(:,t-1);
    prevV = V(:,:,t-1);
    initial = 0;
  end
  if isempty(u)
    [x(:,t), V(:,:,t), LL, VV(:,:,t)] = ...
 kalman_update(A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m), y(:,t), prevx, prevV, 'initial', initial);
  else
    if isempty(ndx)
      [x(:,t), V(:,:,t), LL, VV(:,:,t)] = ...
   kalman_update(A(:,:,m), C(:,:,m), Q(:,:,m), R(:,:,m), y(:,t), prevx, prevV, ... 
   'initial', initial, 'u', u(:,t), 'B', B(:,:,m));
    else
      i = ndx{t};
      % copy over all elements; only some will get updated
      x(:,t) = prevx;
      prevP = inv(prevV);
      prevPsmall = prevP(i,i);
      prevVsmall = inv(prevPsmall);
      [x(i,t), smallV, LL, VV(i,i,t)] = ...
   kalman_update(A(i,i,m), C(:,i,m), Q(i,i,m), R(:,:,m), y(:,t), prevx(i), prevVsmall, ...
   'initial', initial, 'u', u(:,t), 'B', B(i,:,m));
      smallP = inv(smallV);
      prevP(i,i) = smallP;
      V(:,:,t) = inv(prevP);
    end    
  end
  loglik = loglik + LL;
end