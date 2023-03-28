function [DX, DY] = phaseportrait2d(f_handle, X, Y, dim_u)
    [I, J] = size(X);
    DX = zeros(I,J);
    DY = zeros(I,J);
    u = zeros(dim_u, 1);
    
    for i = 1:I
        for j = 1:J
            x = [X(i,j); Y(i,j)];
            f = f_handle(x, u);
            f = f / norm(f); 
            DX(i,j) = f(1); 
            DY(i,j) = f(2);
        end
    end
end