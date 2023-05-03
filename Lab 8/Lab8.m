%% Laboratorio 8
% PARAMETROS
R1=1000;
R3=10000;
R2=R3;
C1=1e-6;
C2=0.1e-6;
C3=10e-6;
e=2.718228;
%modelo 
b_0 = 1/(R1*R2*R3*C1*C2*C3);
a_0 = b_0;
a_2 = 1/(C1*R1) + 1/(C1*R2) + 1/(C3*R3) + 1/(C3*R2); %b0
a_1 = 1/(C2*C3*R2*R3) + 1/(C1*C3*R2*R3) + 1/(C1*C3*R1*R3) + 1/(C1*C3*R1*R2); %c0
dem = [1, a_2, a_1, b_0];
G0 = tf(a_0, dem);
%ecuaciones diferenciales
A_3 = [-(R1+R2)/(R1*R2*C1), 1/(R2*C1),-1/(R2*C1);0,0,-1/(R3*C2);-1/(R2*C3),1/(R2*C3),-(R2+R3)/(R2*R3*C3)];
B_3 = [1/(R1*C1);0;0];
C_3 = [0,1,0];
C_new = eye(3);
D_3 = zeros(3,1);
sistema_original = ss(A_3, B_3, C_3, 0);

%% PRIMERA PARTE
% INCISO 1
%simulink 
% INCISO 2
Control_matrix = ctrb(A_3, B_3);
Controlable = rank(Control_matrix);

if (length(A_3) - rank(Controlable))==0
    disp("controlable"); 
else 
    disp("No es controlable");
end
%-------------------------------------------------------------------------
%numeracion de inciso
inciso=4;
disp("el inciso");
disp(inciso);
if (inciso==2)
    Q = [1,0,0,0;...
        0,1,0,0;...
        0,0,1,0;...
        0,0,0,1];
end
if (inciso==3)
    Q = [1,0,0,0;...
        0,1,0,0;...
        0,0,1000,0;...1000
        0,0,0,pi*30000*3];
end
if(inciso==4)
    Q = [10,0,0,0;...
        0,5000000,0,0;...
        0,0,80000*e*pi,0;...
        0,0,0,pi*300000000000*3];
end
    
R = 1;
[Klqi, S, Closed_loop_poles] = lqi(sistema_original, Q, R);

%% segunda parte 
p = 10*[-1100, -340-90i, -340+90i]; %-1100, -100-100i,-100+100i valor inicial
%valor óptimo -1100, -340-210i, -340+210i
%valor inciso 12 -1100, -340-200i, -340+200i
K = place(A_3, B_3, p);
L = place(A_3' , C_3' , p)';
Acl = A_3 - B_3*K;
sistema_mejorado = ss(Acl,B_3,C_3,0);
%%
disp("Q");
disp(Q);
disp("R");
disp(R);
disp("RL");
disp(1);
disp("K");
disp(K);
disp("L");
disp(L);
disp("Klqi");
disp(Klqi);