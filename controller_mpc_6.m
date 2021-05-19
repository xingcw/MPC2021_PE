% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_6(Q,R,T,N,~)
% controller variables
persistent param yalmip_optimizer Aaug Baug Caug L M P
% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,T,N);
    Aaug = [param.A,param.Bd; zeros(3), eye(3)]; 
    Baug = [param.B; zeros(3)];
    Caug = [param.C_ref, zeros(3)];
    P = [0.8, 0.8, 0.8, 0.0, 0.0, 0.0];
    L = (place(Aaug', Caug', P))';
    M = [param.A - eye(3), param.B; eye(3), zeros(3)];
end

X_input = T - param.Ts;

% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(X_input, param.Ts, param.ps);
if (errorcode ~= 0)
    warning('MPC6 infeasible');
end

disp(errorcode);
p = u_mpc{1} + param.ps;

% observer update
y = T;
estimator = Aaug*[param.x_hat;param.d_hat] +...
            L*(y-Caug*[param.x_hat;param.d_hat])+...
            Baug*p;
param.x_hat = estimator(1:3);
param.d_hat = estimator(4:6);

% set point update
steady_state = [param.A - eye(3), param.B; eye(3), zeros(3)]...
    \ [-param.Bd * param.d_hat; param.T_sp];
param.Ts = steady_state(1:3);
% param.Ts = param.T_sp;
param.ps = steady_state(4:6);
% param.ps = param.p_sp;


% disp(y);
% disp(param.x_hat);
% disp(y - param.x_hat);
disp(param.d_hat);

end

function [param, yalmip_optimizer] = init(Q,R,T,N)
% get basic controller parameters
param = compute_controller_base_parameters;
% get terminal cost
[P_f,~,~] = idare(param.A, param.B, Q, R);
% get terminal set
[A_x,b_x] = compute_X_LQR(Q,R);
% design disturbance observer

% init state and disturbance estimate variables
param.Ts = param.T_sp;
param.ps = param.p_sp;

% param.x_hat = param.T_sp;
param.x_hat = T;
param.d_hat = param.d;
% param.d_hat = [3.555e4, 0, 0]';

% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
xs = sdpvar(nx,1,'full');
us = sdpvar(nu,1,'full');
Ucons = param.Pcons - [us, us];
Xcons = param.Tcons - [xs, xs];
umin = Ucons(:, 1);
umax = Ucons(:, 2);
xmin = Xcons(:, 1);
xmax = Xcons(:, 2);

objective = 0;
constraints = [];

objective = objective + X{1}' * Q * X{1} + U{1}' * R * U{1};
constraints = [X{2} == param.A * X{1} + param.B * U{1},...
               xmin <= X{1} <= xmax, umin<= U{1}<=umax];

for k = 2:N-1
    constraints = [constraints, umin <= U{k} <= umax, xmin <= X{k} <= xmax,...
                   X{k+1} == param.A * X{k} + param.B * U{k}];
    objective = objective + X{k}' * Q * X{k} + U{k}' * R * U{k};
end

constraints = [constraints, A_x * X{N} <= b_x];
objective = objective + X{N}' * P_f * X{N};

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,{X{1},xs,us},U);

end