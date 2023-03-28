function [A,B,C,D] = loclin_best(f_handle, h_handle, xe, ue, delta)
    switch nargin
        case 4
            delta = 1e-6; % default delta value
        case 5
            disp(''); % do nothing
        otherwise
            error('Invalid number of inputs');
    end

    %% Inicialización
    % Se obtienen los tamaños de cada uno de los vectores del sistema
    n = length(xe);
    m = length(ue);
    p = length(h_handle(xe,ue));
    % Se inicializan las matrices del sistema LTI que resultarán de la
    % linealización
    A = zeros(n,n);
    B = zeros(n,m);
    C = zeros(p,n);
    D = zeros(p,m);
    
    %% Obtención de jacobianos

    % A = df(x,u)/dx
    for i = 1:n
        dx = zeros(n,1);
        dx(i) = delta/2;
        ff = f_handle(xe+dx, ue);
        fb = f_handle(xe-dx, ue);
        A(:,i) = (ff-fb)/delta;
    end
    
    % B = df(x,u)/du
    for j = 1:m
        du = zeros(m,1);
        du(j) = delta/2;
        ff = f_handle(xe, ue+du);
        fb = f_handle(xe, ue-du);
        B(:,j) = (ff-fb)/delta;
    end
    
    % C = dh(x,u)/dx
    for k = 1:n
        dx = zeros(n,1);
        dx(k) = delta/2;
        hf = h_handle(xe+dx, ue);
        hb = h_handle(xe-dx, ue);
        C(:,k) = (hf-hb)/delta;
    end
    
    % D = dh(x,u)/du
    for l = 1:m
        du = zeros(m,1);
        du(l) = delta/2;
        hf = h_handle(xe, ue+du);
        hb = h_handle(xe, ue-du);
        D(:,l) = (hf-hb)/delta;
    end
        
end