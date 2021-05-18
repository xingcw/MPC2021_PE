% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

figure(2); set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
xlabel('Energy consumption [kWh]'); 
ylabel('Relative norm of steady state deviation');
%...
R = eye(3);
k=15;
dT = 100;
Q_idx = eye(3);
Q= eye(3);
best_power_sum = 0;
best_dT_relative = 0;
for index = 1:n_samples
    clear controller_lqr
    [T, p, ~,~, T_v,p_v] = simulate_building(T0, @controller_lqr, Q_idx, R, scen, 0);
    dT_relative = norm(T_sp-T(:,k))/norm(T_sp-T0);
    power_sum = sum(abs(p), 'all')/1000/60;
    scatter(power_sum,dT_relative,[],[(T_v)*1,~(T_v|p_v)*1,(p_v)*1]);
    hold on
    if ~(T_v || p_v)
        if (power_sum<16) &&(dT_relative<dT)
             dT = dT_relative;
             Q = Q_idx;
             best_power_sum = power_sum;
             best_dT_relative = dT_relative;
        end   
    end
    Q_idx = diag([randi([1,10e6]),randi([1,10e6]),randi([1,10e6])]);
end
% mark the corresponding scatter point for the resulting Q
scatter(best_power_sum, best_dT_relative, [], 'filled', 'r');
end
