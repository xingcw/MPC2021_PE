function [A_x,b_x] = mRPI(A_c, W, N)
    % init first element
    E = W;
    E_cell = cell(N,1);
    E_cell{1} = E;
    % approximate mRPI set
    index=0;
%     figure(1)
    for i=1:1:N
        E_last = E_cell{i};
        E_cell{i+1} = intersect(E_last,Polyhedron(E_last.A*A_c,E_last.b));
        if (mldivide(E_cell{i},E_cell{i+1}).volume<0.00001)
            index = i;
            break;
        end
        
    end
    % Plot for inspection
%     for i=index:-1:1
%         plot(E_cell{i},'Color',[(1-i/N) (1-i/N) (1-i/N)]);
%         hold on;
%     end
%     title('mRPI set')
%     xlabel('x_1')
%     ylabel('x_2')

    A_x = E_cell{index}.A; % Important
    b_x = E_cell{index}.b;
    
end