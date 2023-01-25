% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
% PRE LABORATORIO
% SISTEMAS DE CONTROL 2 - LABORATORIO 1


R1=1000;
R3=10000;
R2=R3;
C1=1e-6;
C2=0.1e-6;
C3=10e-6;

s_hand = tf([C3*R3,1],[C1*R1*C2*R3*C3*R2,(C1*R1*C2*R3 + C1*R1*R3*C3 + C2*R3*C3*R2),(C1*R1 + C2*R3 + R3*C3),1])
% son parecidas

s = tf(linsys1)
% son parecidas
pole(s)
pzmap(s);
%es un sistema estable

linearSystemAnalyzer(s)