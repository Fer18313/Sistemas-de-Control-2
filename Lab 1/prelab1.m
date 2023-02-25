r1=1000;
r2=10000;
r3=10000;
c1=0.000001;
c2=0.0000001;
c3=0.00001;
k1=(r1*r2*3*c1*c2*c3);
k2=c2*(r3*c3*r1+r3*c3*r2+r1*c1*r2+r1*c1*r3);
k3=(r1*c1+c2*(r1+r2+r3));
G= tf([1],[k1 k2 k3 1]);
step(G);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%parte 4

G1 = tf([1e07], [1, 1120, 3.1e04, 1e07]);
KP = 2.6721;
KI = 91.7418;
KD = 0.019204;
N  = 25943.8814;
Ts = 0.0001;
PIDF = pid(KP,KI,KD,1/N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H = c2d(PIDF,Ts,'tustin');
H_P = c2d(G1,Ts,'tustin');
G_T = feedback(H*H_P,1,-1);
figure(1);
step(5*G_T,0.25);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H1 = c2d(PIDF,Ts,'matched');
H_P1 = c2d(G1,Ts,'matched');
G_M = feedback(H1*H_P1,1,-1);
figure(2);
step(5*G_M,0.25);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms 


%syms s z;
%s = (z-1)/(Ts*z);
%G1_2 = 1e07/(s^3 + 1120*s^2 +3.1e04*s + 1e07);
%snew = subs(G1_2,s,z);
%snew_1 = simplify(snew);

BEuler = pid(KP,KI,KD,1/N,Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler');

z=tf("z",Ts);
TfB=(10*z^3) / (2161*z^3 - 5271*z^2 + 4120*z - 1000);
G_B = feedback(TfB*BEuler,1,-1);
figure(3);
step(5*G_B,0.25);