
clear all;
%define the original process transfer function
num = [0.188];
den = [1.09,1];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2.016;
Gp = zpk(z,p,k,'inputdelay',t_delay)


%define the original process transfer function
num = [3];
den = [1,9,27,32,16];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2;
G = zpk(z,p,k,'inputdelay',t_delay)

step(G,Gp)