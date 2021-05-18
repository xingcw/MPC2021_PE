%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')
param = compute_controller_base_parameters;
T_sp = param.T_sp;

% dT0_exmaple = ...
% T0_example = ...


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
% figure(1); set(gcf, 'WindowStyle' ,'docked');
T0 = T_sp;
% simulate_building(T0);

% Tuning of LQR on first initial condition T_init^(1)
T0_1 = T_sp+[-2.25;1.75;0.75];
% [Q_1,R_1] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);
% figure(3); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_lqr, Q_1, R_1, scen1, 1);

% Tuning of LQR on second initial condition T_init^(2)
T0_2 = T_sp+[1.5;2.75;-0.25];
% [Q_2,R_2] = heuristic_LQR_tuning(2500, T0_2, T_sp, scen1);
% figure(4); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_2, @controller_lqr, Q_2, R_2, scen1, 1);
% pause;


%% From LQR to MPC
disp('First MPC'); 
Q_1 = diag([4109479,2064982,2076632]);
R_1 = eye(3);
Q_2 = diag([5792829, 777500, 472315]);
R_2 = eye(3);
% [A_x,b_x] = compute_X_LQR(Q_1,R_1);
% ====================== task 11 ============================
% figure(5); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_11] = simulate_building(T0_1, @controller_mpc_1, Q_1, R_1, scen1, 0);
% figure(6); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_12] = simulate_building(T0_2, @controller_mpc_1, Q_2, R_2, scen1, 0);
% pause;


%% MPC with guarantees
% disp('MPC with guarantees');
% ===================== task 13 =============================
% figure(7); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_21] = simulate_building(T0_1, @controller_mpc_2, Q_1, R_1, scen1, 0);
% figure(8); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_22] = simulate_building(T0_2, @controller_mpc_2, Q_2, R_2, scen1, 0);

% ===================== task 14 =============================
% figure(9); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_31] = simulate_building(T0_1, @controller_mpc_3, Q_1, R_1, scen1, 0);
% figure(10); set(gcf, 'WindowStyle' ,'docked');
% [~, ~, J_32] = simulate_building(T0_2, @controller_mpc_3, Q_2, R_2, scen1, 0);
% pause;


%% Soft-constrained MPC
% disp('Soft-constrained MPC');
% ===================== task 17 =============================
% figure(11); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_mpc_3, Q_1, R_1, scen2, 1);

% ===================== task 18 =============================
% figure(12); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_mpc_4, Q_1, R_1, scen2, 1);

% ===================== task 19 =============================
% figure(13); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_mpc_3, Q_1, R_1, scen4, 1);
% figure(14); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_mpc_4, Q_1, R_1, scen4, 1);

% ===================== task 20 =============================
N=30;
% d = zeros(3,scen2.Nbar+N);
% d(1,37:51) = -0.015*ones(1,15);
% d(2,37:44) = 0.6*ones(1,8);
% d(3,40:45) = -0.3*ones(1,6);
% d(3,45:50) = 0.7*ones(1,6);
% d(3,48:50) = 1*ones(1,3);
% figure(15); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1, @controller_mpc_5, Q_1, R_1, scen2, 1, N, d);
%pause;


%% Offset-free MPC
disp('Offset-free MPC');
% figure(16); set(gcf, 'WindowStyle' ,'docked');
simulate_building(T0_1,@controller_mpc_6,Q_1,R_1,scen1,1);
% figure(17); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1,@controller_mpc_6,Q_1,R_1,scen2,1);
% figure(18); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T0_1,@controller_mpc_6,Q_1,R_1,scen3,1);
% pause;
% 
% 
%% Comparison using forces
% disp('MPC Implementation with FORCES Pro');