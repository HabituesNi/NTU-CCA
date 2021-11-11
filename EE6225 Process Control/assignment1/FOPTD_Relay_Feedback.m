sim('FOPTD_Relay_Feedback_sim');
plot(simout);

clear all;
%define the original process transfer function
num = [3];
den = [1,9,27,32,16];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2;
G = zpk(z,p,k,'inputdelay',t_delay)

%read from the scope output
L = 2.016;
Pu = 7.914;
Kp = 0.188;
h = 1;
a = 0.181;

Wu = 2*pi/Pu;
Ku = 4*h/(pi*a);
T = sqrt((Kp*Ku)^2-1)/Wu;

num = [Kp];
den  = [T,1];
Gn = tf(num,den,'inputdelay',L)
figure,step(G,Gn,35);
% figure,nyquist(G,Gn,{0.0001,0.56});
figure,nyquist(G,{0.0001,0.56});