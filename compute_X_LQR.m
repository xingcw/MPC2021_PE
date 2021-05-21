% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    % get basic controller parameters
    param = compute_controller_base_parameters;
    % implement the X_LQR computation and assign the result
    A = param.A;
    B = param.B;
    [P,~,~] = dare(A,B,Q,R);
    % Discrete-time LQR controller
    K_E = -1*inv(R + B.'*P*B)*B.'*P*A;
    A_c = A+B*K_E;
    A_x = [eye(3);-eye(3);K_E;-K_E];
    b_x = [param.Xcons(:,2);-param.Xcons(:,1);param.Ucons(:,2);-param.Ucons(:,1)];
    % State constraint set
    X = Polyhedron(A_x,b_x);
    
    % hardcode, as only 30 steps for MPC
    N = 30;
    
    X_list = cell(N, 1);
    X_list{1} = X;
    
    index = 0;
    for i=1:1:N
        X_last = X_list{i};
        X_list{i+1} = intersect(X_last,Polyhedron(X_last.A*A_c,X_last.b));
        if (mldivide(X_list{i},X_list{i+1}).volume<0.00001)
            index = i;
            break;
        end
    end
    
    % plot last polyhedron
    plot(X_list{index}, 'color', 'lightgreen');
    hold on;
    
    % plot initial plot
    % initial condition for ex7
    T_init_1 = [-2.25, 1.75, 0.75]';
    
    scatter3(T_init_1(1), T_init_1(2), T_init_1(3), 'r', 'filled');
    init_cond_text = sprintf('init cond1');
    text(T_init_1(1)+0.1, T_init_1(2), T_init_1(3), init_cond_text);
    
    % initial condition for ex8
    T_init_2 = [1.5, 2.75, -0.25]';
    
    scatter3(T_init_2(1), T_init_2(2), T_init_2(3), 'k', 'filled');
    init_cond_text = sprintf('init cond2');
    text(T_init_2(1)+0.1, T_init_2(2), T_init_2(3), init_cond_text);
    
    alpha 0.5;
    title('maximal Invariant Set');
    xlabel('T_VC');
    ylabel('T_F1');
    zlabel('T_F2');
    
    A_x = X_list{index}.A;
    b_x = X_list{index}.b;

end