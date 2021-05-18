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
    [A_x,b_x] = mRPI(A_c,X,30);

end