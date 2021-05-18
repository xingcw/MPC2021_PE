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

function p = controller_mpc_4(Q, R, T, N, ~)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end

% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
if (errorcode ~= 0)
    warning('MPC4 infeasible');
end
p =u_mpc{1}+param.p_sp;
end

function [param, yalmip_optimizer] = init(Q, R, N)
% get basic controller parameters
param = compute_controller_base_parameters;
% get terminal cost
% ...
% get terminal set
[A_x,b_x]=compute_X_LQR(Q,R);
[P,K,G]=idare(param.A,param.B,Q,R,zeros(3),eye(3));
% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
e = sdpvar(repmat(nx,1,N-1),ones(1,N-1),'full');
%v = sdpvar(1,1,'full');
T0 = sdpvar(nx,1,'full');
objective = 0;
constraints = [X{1}==T0];
for k = 1:N-1
    constraints = [constraints,X{k+1}==param.A*X{k}+param.B*U{k},e{k}>=zeros(3,1),(param.Xcons(:,1)-e{k}<=X{k+1})&(X{k+1}<=param.Xcons(:,2)+e{k}),(param.Ucons(:,1)<=U{k})&(U{k}<=param.Ucons(:,2))];
    %constraints = [constraints,X{k+1}==param.A*X{k}+param.B*U{k},(param.Ucons(:,1)<=U{k})&(U{k}<=param.Ucons(:,2))];
    objective = objective +X{k}'*Q*X{k}+U{k}'*R*U{k}+e{k}'*e{k};
end
constraints = [constraints,A_x*X{N}<=b_x];
objective = objective + X{N}'*P*X{N};
ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,T0,{U{1:N-1},e{1:N-1}});
end